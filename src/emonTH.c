#include <stddef.h>

#include "emonTH_saml.h"

#include "driver_ADC.h"
#include "driver_CLK.h"
#include "driver_DMAC.h"
#include "driver_EIC.h"
#include "driver_PORT.h"
#include "driver_RTC.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"
#include "driver_TIME.h"

#include "configuration.h"
#include "dataPack.h"
#include "emonTH.h"
#include "emonTH_assert.h"
#include "periph_DS18B20.h"
#include "periph_HDC2010.h"
#include "periph_SensirionCO2.h"
#include "periph_rfm69.h"
#include "pulse.h"
#include "temperature.h"
#include "util.h"

#define N_STEPS (32u)

typedef struct TransmitOpt_ {
  bool json;
  bool useRFM;
  bool logSerial;
} TransmitOpt_t;

/*************************************
 * Persistent state variables
 *************************************/

static volatile bool     interactiveUart = false;
static volatile uint32_t ledPulseOvf     = 0;
static volatile uint32_t evtPend;
AssertInfo_t             g_assert_info;

/*************************************
 * Static function prototypes
 *************************************/

static void    boardSetup(EmonTHConfigPacked_t *pCfg, size_t *tempNum);
static void    errorFatal(void);
static void    gpioClr(const size_t gpio);
static void    gpioSet(const size_t gpio);
static void    ledPulseOvfIncr(void);
static void    measureExternal(EmonTHDataset_t *pData, const size_t numExt);
static void    measureInternal(EmonTHDataset_t *pData);
static uint8_t readSlideSW(void);
static void    regEnable(const bool dly);
static void    regDisable(void);
static size_t  tempSetup(void);
static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer);
static void txOptions(const EmonTHConfigPacked_t *pCfg, TransmitOpt_t *pOpt);
static void ucSetup(void);
static void interactiveWait(void);

/*************************************
 * Functions
 *************************************/

void uartPuts(const char *s) {
  EMONTH_ASSERT(s);
  uartPutsBlocking(s);
}

void emonTHEventClr(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = ~(1u << evt);
  uint32_t primask   = __get_PRIMASK();
  __disable_irq();
  evtPend &= evtDecode;
  __set_PRIMASK(primask);
}

void emonTHEventSet(const EVTSRC_t evt) {
  /* Disable interrupts during RMW update of event status */
  uint32_t evtDecode = (1u << evt);
  uint32_t primask   = __get_PRIMASK();
  __disable_irq();
  evtPend |= evtDecode;
  __set_PRIMASK(primask);
}

bool emonTHEventTake(const EVTSRC_t evt) {
  uint32_t evtDecode = (1u << evt);
  uint32_t primask   = __get_PRIMASK();

  __disable_irq();
  bool ret = (evtPend & evtDecode) ? true : false;
  evtPend &= ~evtDecode;
  __set_PRIMASK(primask);

  return ret;
}

static void boardSetup(EmonTHConfigPacked_t *pCfg, size_t *tempNum) {
  char strBuffer[8];

  uint8_t swVal = readSlideSW();
  uartPuts("> Node ID: ");
  utilUtoa(strBuffer, (swVal + pCfg->baseCfg.nodeID), ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");

  uartPuts("> Sample time: ");
  utilUtoa(strBuffer, pCfg->baseCfg.reportTime, ITOA_BASE10);
  uartPuts(strBuffer);
  uartPuts("\r\n");

  RFMOpt_t rfmOpt = {.freq    = pCfg->dataTxCfg.rfmFreq,
                     .group   = pCfg->baseCfg.dataGrp,
                     .nodeID  = (swVal + pCfg->baseCfg.nodeID),
                     .paLevel = pCfg->dataTxCfg.rfmPwr};

  uartPuts("> Setting up RFM69... ");
  /* Initialise RFM69 into sleep mode, regardless of its future use */
  if (rfmInit(&rfmOpt)) {
    rfmSetAESKey(RFM_AES_DEF);
    rfmSleep();
    spiDisable();

    uartPuts("Done\r\n");
  } else {
    uartPuts("Failed\r\n");
    errorFatal();
  }
  uartPuts("\r\n");

  /* Configure the pulse input if in use. */
  if (pCfg->pulseCfg.active) {
    pulseInit(pCfg->pulseCfg.timeMask, pCfg->pulseCfg.pu);
  }

  /* Find I2C sensors */
  uartPuts("Finding sensors:\r\n");
  uartPuts("  - HDC2010... ");

  i2cEnable();

  if (hdc2010Setup()) {
    uartPuts("Done\r\n");
  } else {
    uartPuts("Failed");
    errorFatal();
  }

  scd4xDiscover(pCfg->scdCfg.altitude);
  stcc4Discover(pCfg->scdCfg.altitude);

  i2cDisable();

  /* Find any external temperature sensors.  */
  if (pCfg->baseCfg.extTempEn) {
    *tempNum = tempSetup();
  }
}

static void errorFatal(void) {
  while (1) {
    timerDelaySleep_ms(100);
    portPinDrv(PIN_LED, PIN_DRV_TGL);
  }
}

__attribute__((__unused__)) static void gpioClr(const size_t gpio) {
  const size_t pin = (0 == gpio) ? PIN_GPIO0 : PIN_GPIO1;
  portPinDrv(pin, PIN_DRV_CLR);
}

__attribute__((__unused__)) static void gpioSet(const size_t gpio) {
  const size_t pin = (0 == gpio) ? PIN_GPIO0 : PIN_GPIO1;
  portPinDrv(pin, PIN_DRV_SET);
}

static void interactiveWait(void) {
  /* Wait for 5 s with LED flashing @ 2 Hz. Enter configuration mode by pressing
   * any key. */
  uint32_t remain = 4;

  uartPuts("> Press any key within 5 seconds to enter "
           "configuration.\r\n");

  portPinMux(PIN_LED, 0x4);
  portPinCfg(PIN_LED, PORT_PINCFG_PMUXEN, PIN_CFG_SET);

  timerSetupLED(&ledPulseOvfIncr);
  while (ledPulseOvf < (((N_STEPS * 2) * 5)) && !interactiveUart) {
    samlSleepEnter();

    if (!(ledPulseOvf % (N_STEPS * 2)) && (0 != remain)) {
      char strbuf[4] = {0};
      utilUtoa(strbuf, remain--, ITOA_BASE10);
      uartPuts(strbuf);
    } else if (!(ledPulseOvf % (N_STEPS / 2))) {
      uartPuts(".");
    }
  }

  if (interactiveUart) {
    configEnter();
  }

  uartPuts("\r\n\r\n");
  timerSetup();
  portPinCfg(PIN_LED, PORT_PINCFG_PMUXEN, PIN_CFG_CLR);
  portPinDrv(PIN_LED, PIN_DRV_CLR);
}

static void ledPulseOvfIncr(void) { ledPulseOvf++; }

static void measureExternal(EmonTHDataset_t *pData, const size_t numExt) {

  if (stcc4Present()) {

    int16_t  temp = hdc2010ConvTx10(pData->hdcResRaw.temp) / 10;
    uint16_t rh   = hdc2010ConvRHx10(pData->hdcResRaw.humidity) / 10u;

    stcc4SetRHT(temp, rh);
    pData->co2 = stcc4MeasureCO2();
  }
  pData->pulseCnt = pulseGetCount();

  /* Only a single external will be reported, use OEM unused sentinel. */
  if (!numExt) {
    pData->tempExternal[0] = TEMP_ONEWIRE_RAW_UNUSED;
    return;
  }

  /* Default slots to failure. */
  for (size_t i = 0; i < numExt; i++) {
    pData->tempExternal[i] = TEMP_ONEWIRE_RAW_FAILED;
  }

  /* Mark unused slots. */
  for (size_t i = numExt; i < TEMP_MAX_ONEWIRE; i++) {
    pData->tempExternal[i] = TEMP_ONEWIRE_RAW_UNUSED;
  }

  /* DS18B20 conversion takes 750 ms @ 12 bit resolution */
  if (TEMP_OK == tempSampleStart(TEMP_INTF_ONEWIRE, 0)) {
    timerDelaySleep_ms(800u);
    tempSampleRead(TEMP_INTF_ONEWIRE, pData->tempExternal);
  }
}

static void measureInternal(EmonTHDataset_t *pData) {
  HDCResultRaw_t hdcResultRaw = {0};

  /* Start samples in parallel, proceed when complete */
  i2cEnable();
  hdc2010ConversionStart();

  adcSampleTrigger();

  while (!adcSampleReady() || !hdc2010SampleReady()) {
    samlSleepEnter();
  }

  hdc2010SampleGet(&hdcResultRaw);
  i2cDisable();

  pData->battery   = adcGetResult();
  pData->hdcResRaw = hdcResultRaw;
}

static uint8_t readSlideSW(void) {
  uint8_t swPin[2] = {PIN_SW_NODE0, PIN_SW_NODE1};
  uint8_t swVal    = 0;

  /* If the pin has been pulled low, then disable the pull up as the pin already
   * has a defined value. */
  for (size_t i = 0; i < 2u; i++) {
    unsigned int pinVal = portPinValue(swPin[i]);
    swVal |= pinVal << i;
    if (0 == pinVal) {
      portPinCfg(swPin[i], PORT_PINCFG_PULLEN, PIN_CFG_CLR);
    }
  }

  /* Return the bitwise NOT as the switch ON position -> value of 0 */
  return ~swVal & 0x3u;
}

/*! @brief Disable the external boost regulator. */
static void regDisable(void) { portPinDrv(PIN_REG_EN, PIN_DRV_CLR); }

/*! @brief Enable the external boost regulator.
 *  @param [in] dly : apply 128 us delay to allow 3V3 to settle (Figure 27)
 */
static void regEnable(const bool dly) {
  portPinDrv(PIN_REG_EN, PIN_DRV_SET);
  if (dly) {
    timerDelay_us(128);
  }
}

/*! @brief Initialise temperature sensors (OneWire).
 *  @return number of temperature sensors found
 */
static size_t tempSetup(void) { return tempSensorsInit(TEMP_INTF_ONEWIRE, 0); }

static void transmitData(const EmonTHDataset_t *pSrc, const TransmitOpt_t *pOpt,
                         char *txBuffer) {

  if (pOpt->logSerial) {
    samlSleepIdle(); /* Require IDLE for DMA rather than standby */
    size_t   n   = dataPackSerial(pSrc, txBuffer, TX_BUFFER_W, pOpt->json);
    uint32_t len = (n > TX_BUFFER_W) ? TX_BUFFER_W : (uint32_t)n;
    uartPutsNonBlocking(txBuffer, len);
  }

  if (pOpt->useRFM) {
    spiEnable();
    dataPackPacked(pSrc, (void *)rfmGetBuffer());
    rfmSendBuffer((4 == pSrc->numExtMax) ? sizeof(PackedData_4Ext_t)
                                         : sizeof(PackedData_1Ext_t));
    spiDisable();
  }

  while (!dmacUARTComplete()) {
    samlSleepEnter();
  }

  samlSleepStandby();
}

static void txOptions(const EmonTHConfigPacked_t *pCfg, TransmitOpt_t *pOpt) {
  pOpt->json = pCfg->baseCfg.useJson;

  switch ((TxType_t)pCfg->dataTxCfg.txType) {
  case DATATX_RFM69:
    pOpt->useRFM = true;
    uartDisable();
    break;
  case DATATX_UART:
    pOpt->logSerial = true;
    spiDisable();
    uartDisableRx();
    break;
  case DATATX_BOTH:
    pOpt->useRFM    = true;
    pOpt->logSerial = true;
    uartDisableRx();
    break;
  default:
    pOpt->useRFM = true;
    uartDisable();
  }
}

/*! @brief Setup the microcontroller. Must be called once at startup. */
static void ucSetup(void) {
  /* Start the boost regulator as early as possible */
  portSetup();
  regEnable(true);

  clkSetup();
  sercomSetup();
  rtcSetup();
  adcSetup();
  eicSetup();
  dmacSetup();

  samlSleepConfigure();
  samlSleepStandby();
}

int main(void) {

  EmonTHDataset_t       dataset               = {0};
  EmonTHConfigPacked_t *pConfig               = 0;
  size_t                tempExtNum            = 0;
  char                  txBuffer[TX_BUFFER_W] = {0};
  uint32_t              txCnt                 = 0;
  TransmitOpt_t         txOpt                 = {0};

  ucSetup();
  eicEnable();

  configFirmwareBoardInfo();

  /* Load configuration values from non-volatile memory (NVM). If the NVM has
   * not been used before then store default configuration.
   */
  pConfig = configLoadFromNVM();

  interactiveWait();

  boardSetup(pConfig, &tempExtNum);
  txOptions(pConfig, &txOpt);
  dataset.numExtMax = pConfig->baseCfg.extTempEn;

  if (scd4xPresent()) {
    rtcEvtReg((RTC_Evt_t){.smpInterval = pConfig->scdCfg.sampleInterval,
                          .evt         = EVT_SCD4x_SAMPLE});
    dataset.co2 = scd4xMeasureCO2();
  }

  /* Discard the first sample and take a real sample immediately */
  measureInternal(&dataset);
  emonTHEventSet(EVT_WAKE_TIMER);

  rtcEnable(pConfig->baseCfg.reportTime);

  while (1) {

    if (emonTHEventTake(EVT_WAKE_TIMER)) {
      regEnable(true);

      measureInternal(&dataset);
      measureExternal(&dataset, tempExtNum);

      transmitData(&dataset, &txOpt, txBuffer);

      timerDelaySleep_ms(1);
      txCnt++;

      /* Flash LED for the first 5 transmissions to provide indication to user
       * that the emonTH3 is active. */
      if (txCnt < 6u) {
        emonTHEventSet(EVT_LED_FLASH);
      }
    }

    if (emonTHEventTake(EVT_SCD4x_SAMPLE)) {
      regEnable(true);

      dataset.co2 = scd4xMeasureCO2();
    }

    if (emonTHEventTake(EVT_LED_FLASH)) {
      portPinDrv(PIN_LED, PIN_DRV_SET);
      timerDelaySleep_ms(500u);
      portPinDrv(PIN_LED, PIN_DRV_CLR);
    }

    regDisable();
    samlSleepEnter();
  }
}

void emonTHInteractiveUartSet(void) { interactiveUart = true; }
