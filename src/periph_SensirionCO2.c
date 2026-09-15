#include <stddef.h>

#include "driver_SERCOM.h"
#include "driver_TIME.h"
#include "emonTH.h"
#include "emonTH_saml.h"
#include "periph_SensirionCO2.h"

#define ADDR7_SCD  0x62u
#define ADDR7_STCC 0x64u

#define SC_COMMAND(name, c_, rwc_, n_, t_, sc_)                                \
  static const SC_Cmd_t name = {                                               \
      .cmd = (c_), .rwc = (rwc_), .n = (n_), .t_wait = (t_), .addr7 = (sc_)};

typedef enum RWC_ { C, CF, R, W } RWC_t;
typedef enum SC_Type_t { TYPE_SCD4x, TYPE_STCC } SC_Type_t;
typedef enum SCD_ID_ { SCD40, SCD41, SCD43, SCD4x_NONE } SCD_ID_t;

typedef enum SC_Reg_ {
  SCD_CMD_ALTITUDE_GET   = 0x2322u,
  SCD_CMD_ALTITUDE_SET   = 0x2427u,
  SCD_CMD_PERIODIC_START = 0x21b1u,
  SCD_CMD_PERSIST_CFG    = 0x3615u,
  SCD_CMD_SAMPLE_READ    = 0xec05u,
  SCD_CMD_SAMPLE_READY   = 0xe4b8u,
  SCD_CMD_SAMPLE_SINGLE  = 0x219du,
  SCD_CMD_SCD_VARIANT    = 0x202fu,
  STCC_CMD_PRESSURE_SET  = 0xe016u,
  STCC_CMD_RECONDITION   = 0x29bcu,
  STCC_CMD_RHT_SET       = 0xe000u,
  STCC_CMD_SAMPLE_READ   = 0xec05u,
  STCC_CMD_SAMPLE_SINGLE = 0x219du,
  STCC_CMD_SLEEP_ENTER   = 0x3650u,
  STCC_CMD_STCC_ID       = 0x365bu
} SC_Reg_t;

typedef enum SC_Resp_ {
  SC_RESP_OK,
  SC_RESP_I2C_FAILED,
  SC_RESP_BADCRC
} SC_Resp_t;

typedef struct SC_Cmd_ {
  SC_Reg_t cmd;    /* Command address */
  RWC_t    rwc;    /* Read, write, or command */
  size_t   n;      /* Number of data bytes (0 for command) */
  uint16_t t_wait; /* Time between command and read transfer (ms) */
  uint8_t  addr7;  /* SCD4x or STCC sensor address */
} SC_Cmd_t;

typedef struct __attribute__((__packed__)) RHT_ {
  uint16_t t;
  uint16_t h;
} RHT_t;

/* ==== SCD4x Commands ==== */
SC_COMMAND(cmdAltitudeGet, SCD_CMD_ALTITUDE_GET, R, 2, 1u, ADDR7_SCD)
SC_COMMAND(cmdAltitudeSet, SCD_CMD_ALTITUDE_SET, W, 2, 1u, ADDR7_SCD)
SC_COMMAND(cmdPeriodicStart, SCD_CMD_PERIODIC_START, C, 0, 0, ADDR7_SCD)
SC_COMMAND(cmdPersistCfg, SCD_CMD_PERSIST_CFG, C, 0, 800u, ADDR7_SCD)
SC_COMMAND(cmdSampleReadSCD, SCD_CMD_SAMPLE_READ, R, 2u, 1u, ADDR7_SCD)
SC_COMMAND(cmdSampleReady, SCD_CMD_SAMPLE_READY, R, 2u, 1u, ADDR7_SCD)
SC_COMMAND(cmdSampleSingleSCD, SCD_CMD_SAMPLE_SINGLE, C, 0, 5000u, ADDR7_SCD)
SC_COMMAND(cmdSCDVariant, SCD_CMD_SCD_VARIANT, R, 2u, 1u, ADDR7_SCD)

/* ==== STCC-4 Commands ==== */
SC_COMMAND(cmdPressureSet, STCC_CMD_PRESSURE_SET, W, 2, 1u, ADDR7_STCC)
SC_COMMAND(cmdRecondition, STCC_CMD_RECONDITION, C, 0, 22000u, ADDR7_STCC)
SC_COMMAND(cmdSampleReadSTCC, STCC_CMD_SAMPLE_READ, R, 2u, 1u, ADDR7_STCC)
SC_COMMAND(cmdSampleSingleSTCC, STCC_CMD_SAMPLE_SINGLE, C, 0, 500u, ADDR7_STCC)
SC_COMMAND(cmdSetRHT, STCC_CMD_RHT_SET, W, 4u, 1u, ADDR7_STCC)
SC_COMMAND(cmdSleepEnter, STCC_CMD_SLEEP_ENTER, C, 0, 1u, ADDR7_STCC)
SC_COMMAND(cmdSTCCId, STCC_CMD_STCC_ID, R, 12u, 1u, ADDR7_STCC)

/* ==== Shared functions ==== */
static SC_Resp_t cmdExecute(const SC_Cmd_t *cmd, uint8_t *pData);
static uint8_t   crcCalc(const uint8_t *pData, const size_t n);
static void      printInfo(const SC_Type_t type);
static uint16_t  readU16BE(const uint8_t *pData);
static void      regRead(const SC_Cmd_t *cmd, uint8_t *pData);
static void      regWrite(const SC_Cmd_t *cmd, const uint8_t *pData);

/* ==== SCD4x functions ==== */
static void     initSCD(const uint16_t altitude);
static uint16_t measureSCD40(void);
static void     powerOff(void);
static void     powerOn(void);

/* ==== STCC functions ==== */
static uint16_t  convAltitude2Pressure(const uint16_t altitude);
static uint16_t  convH2H(const uint16_t h);
static uint16_t  convT2T(const int16_t t);
static SC_Resp_t stccSleepExit(void);

/* Presence */
static SCD_ID_t scdID;
static bool     stccPresent;

static int16_t  tInt = 0; /* Temperature from board sensor */
static uint16_t hInt = 0; /* RH from board sensor */

static uint16_t convAltitude2Pressure(const uint16_t altitude) {
  /* Approximation to ~1.5% under 5 km: 101325 - 12(altitude) */
  const uint32_t P0 = 101325ul;

  /* Table 11 - input to register is Pa / 2 */
  return (uint16_t)(((P0 - ((uint32_t)altitude * 12u)) / 2u) & 0xffffu);
}

static uint16_t convH2H(const uint16_t h) {
  /* Section 3.5 Conversion of Signal Input and Output */
  uint32_t tmpH = (((uint32_t)h + 6u) * ((1u << 16) - 1u)) / 125u;
  return (uint16_t)(tmpH & 0xFFFFu);
}

static uint16_t convT2T(const int16_t t) {
  /* Section 3.5 Conversion of Signal Input and Output */
  uint32_t tmpT = t < 0 ? 0 : (uint32_t)t;
  tmpT          = ((tmpT + 45u) * ((1u << 16) - 1u)) / 175u;
  return (uint16_t)(tmpT & 0xFFFFu);
}

static SC_Resp_t cmdExecute(const SC_Cmd_t *cmd, uint8_t *pData) {
  uint8_t addr8  = cmd->addr7 << 1;
  uint8_t regMSB = (cmd->cmd >> 8) & 0xFF;
  uint8_t regLSB = cmd->cmd & 0xFF;

  if (I2CM_SUCCESS != i2cActivate(addr8)) {
    return SC_RESP_I2C_FAILED;
  }

  i2cDataWrite(regMSB);
  i2cDataWrite(regLSB);

  if (W == cmd->rwc) {
    regWrite(cmd, pData);
    timerDelaySleep_ms(cmd->t_wait);
  } else if (R == cmd->rwc) {
    regRead(cmd, pData);
  } else if (C == cmd->rwc) {
    i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
    timerDelaySleep_ms(cmd->t_wait);
  }

  return SC_RESP_OK;
}

static uint8_t crcCalc(const uint8_t *pData, const size_t n) {
  /* See section 3.12 Checksum Calculation in SCD4x datasheet */

  uint8_t crc = 0xFFu;
  for (size_t i = 0; i < n; i++) {
    crc ^= pData[i];
    for (size_t bit = 8; bit > 0; bit--) {
      if (crc & 0x80u) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

static uint16_t readU16BE(const uint8_t *pData) {
  return ((uint16_t)pData[0] << 8) | pData[1];
}

static void powerOff(void) { portPinDrv(PIN_EXT_EN, PIN_DRV_CLR); }

static void powerOn(void) {
  /* Table 7 - requires max. 30 ms power up time */
  portPinDrv(PIN_EXT_EN, PIN_DRV_SET);
  timerDelaySleep_ms(30);
}

static void initSCD(uint16_t altitude) {
  uint8_t dBuf[2];

  /* Check altitude has been set as configured */
  cmdExecute(&cmdAltitudeGet, dBuf);
  uint16_t currentAltitude = ((uint16_t)dBuf[0] << 8) | dBuf[1];
  if (altitude != currentAltitude) {
    dBuf[0] = altitude & 0xFFu;
    dBuf[1] = altitude >> 8;
    cmdExecute(&cmdAltitudeSet, dBuf);
    cmdExecute(&cmdPersistCfg, NULL);
  }

  if (SCD40 == scdID) {
    cmdExecute(&cmdPeriodicStart, NULL);
  }
}

static uint16_t measureSCD40(void) {
  uint8_t dbuf[2];

  /* the 11 LSBs of of data ready are 0 when not ready */
  do {
    cmdExecute(&cmdSampleReady, dbuf);
  } while (0 == (dbuf[1] & 0x7FF));

  cmdExecute(&cmdSampleReadSCD, dbuf);
  return readU16BE(dbuf);
}

static void printInfo(const SC_Type_t type) {

  if (TYPE_SCD4x == type) {
    uartPuts("  - SCD4x... ");

    switch (scdID) {
    case SCD40:
      uartPuts("SCD40");
      break;
    case SCD41:
      uartPuts("SCD41");
      break;
    case SCD43:
      uartPuts("SCD43");
      break;
    case SCD4x_NONE:
      uartPuts("None");
    }
  }
  uartPuts("\r\n");
}

static void regRead(const SC_Cmd_t *cmd, uint8_t *pData) {
  uint8_t addr8 = (cmd->addr7 << 1) | 0x1u;

  timerDelaySleep_ms(cmd->t_wait);
  if (I2CM_SUCCESS == i2cActivate(addr8)) {
    for (size_t i = 0; i < cmd->n; i = i + 2) {
      pData[i] = i2cDataRead();
      i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
      pData[i + 1u] = i2cDataRead();
      i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
      /* Revisit : discarding CRC for now. */
      (void)i2cDataRead();
      if ((cmd->n - 2u) != i) {
        i2cAck(I2CM_ACK, I2CM_ACK_CMD_CONTINUE);
      }
    }
    i2cAck(I2CM_NACK, I2CM_ACK_CMD_STOP);
  }
}

static void regWrite(const SC_Cmd_t *cmd, const uint8_t *pData) {
  /* All commands are 16 bit, MSB first */
  for (size_t i = 0; i < cmd->n; i = i + 2u) {
    uint8_t word[2] = {pData[i + 1u], pData[i]};

    i2cDataWrite(word[0]);
    i2cDataWrite(word[1]);
    i2cDataWrite(crcCalc(word, sizeof(word)));
  }
  i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);
}

void scd4xDiscover(const uint16_t altitude) {
  uint8_t rxBuf[2];
  scdID = SCD4x_NONE;

  powerOn();

  /* Bits [15:12] of word[0] encode variant (3.10.6) */
  if (SC_RESP_OK == cmdExecute(&cmdSCDVariant, rxBuf))
    switch (rxBuf[1]) {
    case 0x04u:
      scdID = SCD40;
      break;
    case 0x14u:
      scdID = SCD41;
      break;
    case 0x54u:
      scdID = SCD43;
      break;
    default:
      scdID = SCD4x_NONE;
    }

  printInfo(TYPE_SCD4x);

  if (SCD4x_NONE != scdID) {
    initSCD(altitude);
  }

  /* SCD40 does not support power cycling, leave on */
  if (SCD40 != scdID) {
    powerOff();
  }
}

uint16_t scd4xMeasureCO2(void) {

  /* SCD40 does not support power cycled sampling */
  if (scdID == SCD40) {
    return measureSCD40();
  }

  uint8_t co2[2];
  bool    i2cIsEnabled = i2cEnabled();

  if (!i2cIsEnabled) {
    i2cEnable();
  }

  powerOn();

  /* In power cycled mode, need to discard the first sample as junk */
  cmdExecute(&cmdSampleSingleSCD, NULL);
  cmdExecute(&cmdSampleSingleSCD, NULL);
  /* Revisit : check for data ready? */
  cmdExecute(&cmdSampleReadSCD, co2);

  if (!i2cIsEnabled) {
    i2cDisable();
  }

  powerOff();
  return readU16BE(co2);
}

bool scd4xPresent(void) { return scdID != SCD4x_NONE; }

void stcc4Discover(const uint16_t altitude) {
  uint8_t rxBuf[12] = {0};
  stccPresent       = false;

  /* Section 3.4.16 : STCC-4 product ID */
  if (SC_RESP_OK == stccSleepExit()) {
    cmdExecute(&cmdSTCCId, rxBuf);
    uint32_t pid = ((uint32_t)rxBuf[0] << 24) | (uint32_t)rxBuf[1] << 16 |
                   ((uint32_t)rxBuf[2] << 8) | (uint32_t)rxBuf[3];

    if (0x0901018a == pid) {
      stccPresent = true;
    }
  }

  uartPuts("  - STCC...");

  if (stccPresent) {
    uartPuts("\r\n    > Reconditioning... ");

    uint16_t pa = convAltitude2Pressure(altitude);
    cmdExecute(&cmdPressureSet, (uint8_t *)&pa);
    /* Assume >3 hr since last sample on boot so recondition (Section 1.1.13) */
    cmdExecute(&cmdRecondition, NULL);
    cmdExecute(&cmdSleepEnter, NULL);
  }

  uartPuts(stccPresent ? "Done\r\n" : " None\r\n");
}

uint16_t stcc4MeasureCO2(void) {

  uint8_t co2[2]       = {0};
  RHT_t   rht          = {0};
  bool    i2cIsEnabled = i2cEnabled();

  if (!i2cIsEnabled) {
    i2cEnable();
  }

  rht.h = convH2H(hInt);
  rht.t = convT2T(tInt);

  if (SC_RESP_OK == stccSleepExit()) {
    cmdExecute(&cmdSetRHT, (uint8_t *)&rht);
    cmdExecute(&cmdSampleSingleSTCC, NULL);
    cmdExecute(&cmdSampleReadSTCC, co2);
    cmdExecute(&cmdSleepEnter, NULL);
  }

  if (!i2cIsEnabled) {
    i2cDisable();
  }

  return (uint16_t)co2[1] | ((uint16_t)co2[0] << 8);
}

bool stcc4Present(void) { return stccPresent; }

void stcc4SetRHT(const int16_t t, const uint16_t rh) {
  tInt = t;
  hInt = rh;
}

static SC_Resp_t stccSleepExit(void) {
  /* Exit sleep is a write to the STCC-4 address followed by a single 0x00
   * byte. Can confirm wake by reading product ID. Section 3.4.8
   * exit_sleep_mode
   */

  uint8_t addr8 = ADDR7_STCC << 1;

  if (I2CM_SUCCESS != i2cActivate(addr8)) {
    return SC_RESP_I2C_FAILED;
  }
  i2cDataWrite(0x00u);
  i2cAck(I2CM_ACK, I2CM_ACK_CMD_STOP);

  timerDelaySleep_ms(5u);
  return SC_RESP_OK;
}
