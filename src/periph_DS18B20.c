#include <stdbool.h>

#include "emonTH_saml.h"

#include "board_def.h"
#include "driver_PORT.h"
#include "driver_TIME.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"

typedef struct __attribute__((__packed__)) Scratch_ {
  int16_t temp;
  uint8_t th;
  uint8_t tl;
  uint8_t cfg;
  uint8_t res_FF;
  uint8_t res_X;
  uint8_t res_10;
  uint8_t crc;
} Scratch_t;

/* Driver for DS18B20 OneWire temperature sensor
 * https://www.analog.com/media/en/technical-documentation/data-sheets/DS18B20.pdf
 *
 * OneWire search algorithm adapted from:
 * https://www.analog.com/en/app-notes/1wire-search-algorithm.html
 */

/* Device address table */
static DS18B20_Slot_t slots[TEMP_MAX_ONEWIRE];
static volatile bool  rstPulseComplete = false;

/* OneWire functions & state variables */
static uint8_t      calcCRC8(const uint8_t crc, const uint8_t value);
static bool         oneWireFirst(void);
static bool         oneWireNext(void);
static void         oneWirePwrOff(void);
static void         oneWirePwrOn(void);
static unsigned int oneWireReadBit(void);
static void         oneWireReadBytes(void *pDst, const size_t n);
static bool         oneWireReset(void);
static bool         oneWireSearch(void);
static void         oneWireWriteBit(unsigned int bit);
static void         oneWireWriteBytes(const void *pSrc, const size_t n);
static void         setRstPulseComplete(void);

static uint64_t ROM_NO;
static int32_t  lastDiscrepancy;
static int32_t  lastFamilyDiscrepancy;
static bool     lastDeviceFlag;

static uint8_t calcCRC8(const uint8_t crc, const uint8_t value) {
  static const uint8_t dscrc_table[] = {
      0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
      65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
      130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
      222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
      29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
      102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
      165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
      249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
      58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
      15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
      204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
      144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
      83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
      40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
      235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
      183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
      116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
      53};

  return dscrc_table[crc ^ value];
}

/*! @brief Find the first device on the 1-Wire bus
 *  @return true: device found, ROM number in ROM_NO buffer
 *          false: no devices present
 */
static bool oneWireFirst(void) {
  /* Reset the search state */
  lastDiscrepancy       = 0;
  lastDeviceFlag        = false;
  lastFamilyDiscrepancy = 0;

  return oneWireSearch();
}

static void oneWirePwrOff(void) { portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_CLR); }

static void oneWirePwrOn(void) {
  portPinDrv(PIN_ONEWIRE_PWR, PIN_DRV_SET);
  timerDelaySleep_us(250);
}

/*! @brief Find the next device on the 1-Wire bus
 *  @return true: device found, ROM number in ROM_NO buffer
 *          false: device not found, end of search
 */
static bool oneWireNext(void) { return oneWireSearch(); }

static unsigned int oneWireReadBit(void) {
  unsigned int result = 0;

  __disable_irq();
  portPinDir(PIN_ONEWIRE, PIN_DIR_OUT);
  timerDelay_us(5);
  portPinDir(PIN_ONEWIRE, PIN_DIR_IN);
  /* Max 15 us for read slot; leave 3 us slack */
  timerDelay_us(12u - 5);
  result = portPinValue(PIN_ONEWIRE);
  __enable_irq();

  /* Wait for the end of the read slot, t_RDV */
  timerDelay_us(60u);

  return result;
}

static void oneWireReadBytes(void *pDst, const size_t n) {
  EMONTH_ASSERT(pDst);

  uint8_t *pData = (uint8_t *)pDst;

  for (size_t i = 0; i < n; i++) {
    *pData = 0;
    for (size_t j = 0; j < 8; j++) {
      /* Data received LSB first */
      *pData |= (oneWireReadBit() << j);
    }
    pData++;
  }
}

static bool oneWireReset(void) {
  /* t_RSTL (min) = 480 us
   * t_RSTH (min) = 480 us
   * t_PDHIGH (max) = 60 us
   * t_PDLOW (max) = 240 us
   */

  bool presence = false;

  portPinDrv(PIN_ONEWIRE, PIN_DRV_CLR);
  portPinDir(PIN_ONEWIRE, PIN_DIR_OUT);

  timerDelaySleep_us(512u);

  portPinDir(PIN_ONEWIRE, PIN_DIR_IN);
  /* Wait 48+20 us (wake up) to ensure t_PDHIGH has elapsed, then wait the full
   * t_RSTH time +25 us slack to complete the reset sequence.
   */
  timerDelaySleep_us(75);

  /* Set the async timer and poll the 1-Wire input for LOW from device. */
  rstPulseComplete = false;
  timerDelaySleepAsync_us(450u, &setRstPulseComplete);
  while (!rstPulseComplete) {
    if (0 == portPinValue(PIN_ONEWIRE)) {
      presence = true;
    }
  }

  return presence;
}

static bool oneWireSearch(void) {
  /* Initialise for search */
  static const uint8_t cmdSearchRom = 0xF0u;

  uint32_t searchDirection = 0;
  int32_t  idBitNumber     = 1;
  int32_t  lastZero        = 0;
  uint8_t  romByteMask     = 1;
  bool     searchResult    = false;
  uint8_t  idBit           = 0;
  uint8_t  cmpidBit        = 0;
  uint8_t  crc8            = 0;
  uint8_t *romBuffer       = (uint8_t *)&ROM_NO;

  /* If the last call was not the last one... */
  if (!lastDeviceFlag) {
    /* ... reset the OneWire bus... */
    if (!oneWireReset()) {
      /* Reset the search */
      lastDiscrepancy       = 0;
      lastDeviceFlag        = false;
      lastFamilyDiscrepancy = 0;
      return 0;
    }

    /* ...issue the search command...*/
    oneWireWriteBytes(&cmdSearchRom, 1u);

    /* ...and commence the search! */
    for (size_t i = 0; i < 64; i++) {
      idBit    = oneWireReadBit();
      cmpidBit = oneWireReadBit();

      /* Check for no devices on OneWire */
      if (idBit && cmpidBit) {
        break;
      }

      if (idBit != cmpidBit) {
        searchDirection = idBit;
      } else {
        /* If this discrepancy is before the last discrepancy on a previous
         * next then pick the same as last time
         */
        searchDirection = (idBitNumber < lastDiscrepancy)
                              ? ((*romBuffer & romByteMask) > 0)
                              : (idBitNumber == lastDiscrepancy);

        /* If 0 was picked, record its position */
        if (0 == searchDirection) {
          lastZero = idBitNumber;
          /* and check for last discrepancy in Family */
          if (lastZero < 9) {
            lastFamilyDiscrepancy = lastZero;
          }
        }
      }

      /* Set or clear the bit in the ROM byte number with mask */
      if (0 == searchDirection) {
        *romBuffer &= ~romByteMask;
      } else {
        *romBuffer |= romByteMask;
      }

      /* Serial number search direction bit */
      oneWireWriteBit(searchDirection);
      idBitNumber++;
      romByteMask <<= 1;

      /* When the mask is 0, go to new serial number byte and reset */
      if (0 == romByteMask) {
        crc8 = calcCRC8(crc8, *romBuffer);
        romBuffer++;
        romByteMask = 1;
      }
    }
  }

  /* If the search was successful... */
  if (!((65 > idBitNumber) || (0 != crc8))) {
    lastDiscrepancy = lastZero;
    searchResult    = true;

    /* Check for last device */
    if (0 == lastDiscrepancy) {
      lastDeviceFlag = true;
    }
  }

  return searchResult;
}

static void oneWireWriteBit(unsigned int bit) {
  /* See timing diagrams in Figure 16. Interrupts are disabled in sections
   * where too long would break the OneWire protocol. At the end of a bit
   * transmission, a pending interrupt may be serviced, but this will only
   * extend the interbit timing, with no affect on the protocol.
   */

  __disable_irq();
  portPinDir(PIN_ONEWIRE, PIN_DIR_OUT);
  timerDelay_us(5);
  if (bit) {
    portPinDir(PIN_ONEWIRE, PIN_DIR_IN);
  }
  timerDelay_us(75u - 5);
  portPinDir(PIN_ONEWIRE, PIN_DIR_IN);
  __enable_irq();
  timerDelay_us(5u);
}

static void oneWireWriteBytes(const void *pSrc, const size_t n) {
  uint8_t *pData = (uint8_t *)pSrc;
  for (size_t i = 0; i < n; i++) {
    uint8_t byte = *pData++;
    for (size_t j = 0; j < 8; j++) {
      oneWireWriteBit((byte & 0x1));
      byte >>= 1;
    }
  }
}

size_t ds18b20InitSensors(DS18B20_Slot_t *pSlot) {

  size_t deviceCount  = 0;
  bool   searchResult = false;

  oneWirePwrOn();

  searchResult = oneWireFirst();

  /* DS18B20's family ID is 0x28 in LSB */
  while (searchResult && (deviceCount < TEMP_MAX_ONEWIRE)) {
    if (0x28u == (ROM_NO & (uint64_t)0xFFu)) {
      pSlot[deviceCount].active  = true;
      pSlot[deviceCount].address = ROM_NO;
      deviceCount++;
    }

    searchResult = oneWireNext();
  }

  for (size_t i = 0; i < deviceCount; i++) {
    slots[i].active  = true;
    slots[i].address = pSlot[i].address;
  }

  oneWirePwrOff();
  return deviceCount;
}

void ds18b20PowerOff(void) { oneWirePwrOff(); }

void ds18b20PowerOn(void) { oneWirePwrOn(); }

TempStatus_t ds18b20StartSample(void) {
  static const uint8_t cmds[2] = {0xCC, 0x44};

  oneWirePwrOn();

  /* Check for presence pulse before continuing */
  if (!oneWireReset()) {
    return TEMP_NO_SENSORS;
  }

  oneWireWriteBytes(cmds, 2u);
  return TEMP_OK;
}

DS18B20_Res_t ds18b20ReadSample(const unsigned int dev) {
  static const uint8_t CMD_MATCH_ROM    = 0x55;
  static const uint8_t CMD_SCRATCH_READ = 0xBE;
  static const int16_t DS_T85DEG        = 1360;
  static const int16_t DS_TNEG55DEG     = -880;
  static const int16_t DS_T125DEG       = 2000;

  const uint64_t *addrDev = &slots[dev].address;
  Scratch_t       scratch = {0};
  const uint8_t  *si      = (uint8_t *)&scratch;
  uint8_t         crcDS   = 0;
  DS18B20_Res_t   tempRes = {0};

  /* Initialise with the bad-temperature sentinel. */
  tempRes.status = TEMP_OK;
  tempRes.temp   = TEMP_ONEWIRE_RAW_FAILED;

  /* Check for presence pulse before continuing */
  if (!oneWireReset()) {
    tempRes.status = TEMP_NO_SENSORS;
    return tempRes;
  }

  oneWireWriteBytes(&CMD_MATCH_ROM, 1u);
  oneWireWriteBytes(addrDev, 8u);
  oneWireWriteBytes(&CMD_SCRATCH_READ, 1u);
  oneWireReadBytes(&scratch, sizeof(scratch));

  /* Check CRC for received data */
  for (size_t i = 0; i < (sizeof(scratch) - 1); i++) {
    crcDS = calcCRC8(crcDS, si[i]);
  }

  if (crcDS != scratch.crc) {
    tempRes.status = TEMP_BAD_CRC;
    return tempRes;
  }

  /* The DS18B20's configuration register must not be zero (Figure 10) */
  if (!scratch.cfg) {
    tempRes.status = TEMP_BAD_SENSOR;
    return tempRes;
  }

  /* Check for spurious 85°C reading. This could be caused by e.g. a power
   * glitch after the sample was requested. */
  if ((0x0C == scratch.res_X) && (DS_T85DEG == scratch.temp)) {
    tempRes.status = TEMP_BAD_SENSOR;
    return tempRes;
  }

  /* Ensure in range: 125°C >= T >= -55°C */
  if ((DS_TNEG55DEG > scratch.temp) || (DS_T125DEG < scratch.temp)) {
    tempRes.status = TEMP_OUT_OF_RANGE;
    return tempRes;
  }

  tempRes.temp = scratch.temp;
  return tempRes;
}

static void setRstPulseComplete(void) { rstPulseComplete = true; }
