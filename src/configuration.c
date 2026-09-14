#include <string.h>

#include "emonTH_assert.h"
#include "emonTH_saml.h"

#include "driver_NVM.h"
#include "driver_PORT.h"
#include "driver_RTC.h"
#include "driver_SAML.h"
#include "driver_SERCOM.h"

#include "configuration.h"
#include "emonTH.h"
#include "emonTH_build_info.h"
#include "util.h"

/*************************************
 * Types
 *************************************/

typedef enum {
  RCAUSE_SYST  = 0x40,
  RCAUSE_WDT   = 0x20,
  RCAUSE_EXT   = 0x10,
  RCAUSE_BOD33 = 0x04,
  RCAUSE_BOD12 = 0x02,
  RCAUSE_POR   = 0x01
} RCAUSE_t;

typedef struct CmdArgs_ {
  char  *argv[10];
  size_t argc;
} CmdArgs_t;

/*************************************
 * Prototypes
 *************************************/

static bool        configCheckUnsaved(void);
static bool        configDatalog(void);
static void        configDefault(void);
static bool        configExtTempMax(void);
static bool        configJSON(void);
static bool        configProcessCmd(void);
static bool        configPulse(void);
static void        configRestore(void);
static bool        configRF433(void);
static bool        configRFM(void);
static bool        configRFPower(void);
static void        configSaveToNVM(void);
static bool        configCO2(void);
static bool        configUART(void);
static const char *getLastReset(void);
static void        inBufferClear(void);
static CmdArgs_t   inBufferTok(void);
static void        printInvalidVal(void);
static void        printSettingCO2(void);
static void        printSettingJSON(void);
static void        printSettingPeriod(void);
static void        printSettingPulse(void);
static void        printSettingRF(void);
static void        printSettingRFFreq(void);
static void        printSettingUART(void);
static void        printSettings(void);
static void        printSettingsHR(void);
static void        printSettingsKV(void);
static void        putUint(const uint32_t u);
static void        putUniqueID(void);
static bool        requireExactArgs(const size_t n);
static void        uartPutsError(const char *msg);

/*************************************
 * Local variables
 *************************************/

#define IN_BUFFER_W (64u)

static char            inBuffer[IN_BUFFER_W];
static volatile char   inBufferVolatile[IN_BUFFER_W];
static volatile size_t inBufferIdx = 0;
static volatile bool   cmdPending  = false;

static EmonTHConfigPacked_t config    = {0};
static EmonTHConfigPacked_t configNVM = {0};

static CmdArgs_t cmdArgs       = {0};
static bool      unsavedChange = false;

static bool configCheckUnsaved(void) {
  return (0 != memcmp(&config, &configNVM, sizeof(config)));
}

static bool configDatalog(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if (convU.val.u16 < 5u) {
    uartPutsError("sample period must be greater than 4 s\r\n");
    return false;
  }
  if (convU.val.u32 > RTC_PERIOD_MAX_SECONDS) {
    uartPutsError("sample period exceeds RTC maximum");
    return false;
  }
  if (convU.val.u32 > config.scdCfg.sampleInterval) {
    uartPutsError("sample period must not exceed CO2 sample interval");
    return false;
  }

  config.baseCfg.reportTime = convU.val.u16;
  printSettingPeriod();
  return true;
}

/*! @brief Set all configuration values to defaults */
static void configDefault(void) {
  (void)memset(&config, 0, sizeof(config));

  config.baseCfg.nodeID     = NODE_ID_DEF;       // Node ID
  config.baseCfg.dataGrp    = NETWORK_GROUP_DEF; // Group for OEM
  config.baseCfg.reportTime = WAKE_PERIOD_DEF;   // Time between reports
  config.baseCfg.useJson    = true;              // JSON format for serial
  config.baseCfg.extTempEn  = TEMP_NUM_DEF;      // Max num external sensors

  config.dataTxCfg.txType  = (uint8_t)DATATX_RFM69; // RFM only
  config.dataTxCfg.rfmPwr  = 0x1Fu;                 // +13 dBm (REVISIT)
  config.dataTxCfg.rfmFreq = 3u;                    // 433.92 MHz

  config.pulseCfg.active   = false; // Pulse channel inactive
  config.pulseCfg.pu       = 1;     // Pull down
  config.pulseCfg.timeMask = 25u;   // 25 ms minimum between pulses

  config.scdCfg.altitude       = 0u;   // Sea level
  config.scdCfg.sampleInterval = 600u; // 10 minute CO2 sampling
}

static bool configExtTempMax(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }

  /* Must be 0, 1 or 4 */
  if ((0 != convU.val.u8) && (1u != convU.val.u8) && (4u != convU.val.u8)) {
    uartPutsError("must be in [0,1,4]\r\n");
    return false;
  }

  config.baseCfg.extTempEn = convU.val.u8;
  return true;
}

static bool configJSON(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if (convU.val.u8 > 1u) {
    printInvalidVal();
    return false;
  }

  config.baseCfg.useJson = (bool)convU.val.u8;
  printSettingJSON();
  return true;
}

static bool configNodeID(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if ((convU.val.u8 < 1u) || (convU.val.u8 > 60u)) {
    uartPutsError("ID must be [1..60]\r\n");
    return false;
  }

  config.baseCfg.nodeID = convU.val.u8;

  printSettingRF();
  return true;
}

static bool configPulse(void) {
  ConvUint_t convU;
  bool       active   = 0;
  uint8_t    pu       = 0;
  uint8_t    timeMask = 0;

  if ((cmdArgs.argv[0][1] != '\0') ||
      ((2u != cmdArgs.argc) && (4u != cmdArgs.argc))) {
    uartPutsError("expected active, pull, and period");
    return false;
  }

  convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if (convU.val.u8 > 1u) {
    printInvalidVal();
    return false;
  }
  active = (bool)convU.val.u8;

  if (!active) {
    config.pulseCfg.active = false;
    printSettingPulse();
    return true;
  }

  if (4u != cmdArgs.argc) {
    uartPutsError("expected pull and period");
    return false;
  }

  const char pull = cmdArgs.argv[2][0];

  bool validPull = ('d' == pull) || ('u' == pull) || ('n' == pull);

  if (validPull) {
    switch (pull) {
    case 'd':
      pu = 1u;
      break;
    case 'u':
      pu = 2u;
      break;
    case 'n':
      pu = 0;
    }
  } else {
    uartPutsError("invalid pull configuration (u, d, n).");
    return false;
  }

  convU = utilAtoui(cmdArgs.argv[3], ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  timeMask = convU.val.u8;

  config.pulseCfg.active   = true;
  config.pulseCfg.pu       = pu;
  config.pulseCfg.timeMask = timeMask;

  printSettingPulse();
  return true;
}

static void configRestore(void) {
  if (requireExactArgs(1u)) {
    if ('\0' == cmdArgs.argv[0][1]) {
      configDefault();
      uartPuts("> Restored default values.\r\n");
    } else if ('s' == cmdArgs.argv[0][1]) {
      memcpy(&config, &configNVM, sizeof(config));
      uartPuts("> Restored saved values.\r\n");
    } else {
      uartPutsError("invalid option.");
    }
  } else {
    uartPutsError("unexpected arguments.");
  }
}

static bool configRF433(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);

  if (!convU.valid || (convU.val.u8 > 1u)) {
    printInvalidVal();
    return false;
  }

  /* Only applies to 433 MHz ISM band */
  if (!((config.dataTxCfg.rfmFreq == 2u) || (config.dataTxCfg.rfmFreq == 3u))) {
    uartPutsError("only for 433 MHz ISM\r\n");
    return false;
  }

  config.dataTxCfg.rfmFreq = (0 == convU.val.u8) ? 3u : 2u;

  printSettingRF();
  return true;
}

static bool configRFM(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if (convU.val.u8 > 1u) {
    printInvalidVal();
    return false;
  }
  if (convU.val.u8) {
    config.dataTxCfg.txType |= (1u << 0);
  } else {
    config.dataTxCfg.txType &= ~(1u << 0);
  }

  printSettingRF();
  return true;
}

static bool configRFPower(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }
  if ((convU.val.u8 == 0) || (convU.val.u8 > 31)) {
    uartPutsError("power must be in range [1..31]\r\n");
    return false;
  }

  config.dataTxCfg.rfmPwr = convU.val.u8;

  printSettingRF();
  return true;
}

static bool configCO2(void) {

  if (2u > cmdArgs.argc) {
    uartPutsError("expected at least altitude");
    return false;
  } else if (3u < cmdArgs.argc) {
    uartPutsError("unexpected argument");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[1], ITOA_BASE10);
  if (!convU.valid) {
    uartPutsError("invalid altitude.");
    return false;
  } else {
    config.scdCfg.altitude = convU.val.u16;
  }

  if (3u == cmdArgs.argc) {

    convU = utilAtoui(cmdArgs.argv[2], ITOA_BASE10);
    if (!convU.valid) {
      uartPutsError("invalid sample time.");
      return false;
    }
    if (convU.val.u32 < config.baseCfg.reportTime) {
      uartPutsError("sample interval must be at least the report period");
      return false;
    }
    if (convU.val.u32 > 0xFFFFu) {
      uartPutsError("sample interval exceeds storage maximum");
      return false;
    }
    config.scdCfg.sampleInterval = convU.val.u16;
  }

  printSettingCO2();
  return true;
}

static bool configUART(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  ConvUint_t convU = utilAtoui(cmdArgs.argv[0] + 1, ITOA_BASE10);
  if (!convU.valid) {
    printInvalidVal();
    return false;
  }

  if (convU.val.u8 > 1u) {
    printInvalidVal();
    return false;
  }

  if (convU.val.u8) {
    config.dataTxCfg.txType |= (1u << 1);
  } else {
    config.dataTxCfg.txType &= ~(1u << 1);
  }

  printSettingUART();
  return true;
}

/*! @brief Get the last reset cause (21.8.1)
 *  @return null-terminated string with the last cause.
 */
static const char *getLastReset(void) {
  const RCAUSE_t lastReset = (RCAUSE_t)RSTC->RCAUSE.reg;
  switch (lastReset) {
  case RCAUSE_SYST:
    return "Reset request";
    break;
  case RCAUSE_WDT:
    return "Watchdog timeout";
    break;
  case RCAUSE_EXT:
    return "External reset";
    break;
  case RCAUSE_BOD33:
    return "3V3 brownout";
    break;
  case RCAUSE_BOD12:
    return "1V2 brownout";
    break;
  case RCAUSE_POR:
    return "Power on cold reset";
    break;
  }
  return "Unknown";
}

/*! @brief Fetch part of the SAML's 128-bit unique ID.
 *  @param [in] idx : index of 32-bit word (0..3)
 *  @return 32-bit word from index
 */
uint32_t getUniqueID(const size_t idx) {
  /* Section 10.3 Serial Number */
  const uint32_t id_addr_lut[4] = {0x0080A00C, 0x0080A040, 0x0080A044,
                                   0x0080A048};
  return *(volatile uint32_t *)id_addr_lut[idx];
}

static void inBufferClear(void) {
  inBufferIdx = 0;
  cmdArgs     = (CmdArgs_t){0};

  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    inBufferVolatile[i] = 0;
    inBuffer[i]         = 0;
  }
}

static CmdArgs_t inBufferTok(void) {
  CmdArgs_t result = {0};
  bool      inTok  = false;

  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    if ('\0' == inBuffer[i]) {
      break;
    }
    if (' ' == inBuffer[i]) {
      inBuffer[i] = '\0';
      inTok       = false;
      continue;
    }

    if (!inTok) {
      if (result.argc < (sizeof(result.argv) / sizeof(result.argv[0]))) {
        result.argv[result.argc++] = &inBuffer[i];
      }
      inTok = true;
    }
  }

  return result;
}

static void printInvalidVal(void) { uartPutsError("invalid value\r\n"); }

static void printSettingCO2(void) {
  uartPuts("co2_altitude = ");
  putUint(config.scdCfg.altitude);
  uartPuts(", scd4x_period = ");
  putUint(config.scdCfg.sampleInterval);
  uartPuts("\r\n");
}

static void printSettingJSON(void) {
  uartPuts("json = ");
  uartPuts(config.baseCfg.useJson ? "on" : "off");
  uartPuts("\r\n");
}

static void printSettingPeriod(void) {
  uartPuts("report time = ");
  putUint(config.baseCfg.reportTime);
  uartPuts("\r\n");
}

static void printSettingPulse(void) {
  uartPuts("pulse = ");
  uartPuts(config.pulseCfg.active ? "on" : "off");

  const uint8_t pu = config.pulseCfg.pu;
  uartPuts(", pull = ");
  uartPuts((0 == pu) ? "none" : (1 == pu ? "down" : "up"));
  uartPuts(", period = ");
  putUint(config.pulseCfg.timeMask);
  uartPuts("\r\n");
}

static void printSettingRF(void) {
  uartPuts("RF = ");
  uartPuts(config.dataTxCfg.txType & 0x01 ? "on" : "off");
  uartPuts(", rfBand = ");
  printSettingRFFreq();
  uartPuts(" MHz, ");
  uartPuts("rfGroup = ");
  putUint(config.baseCfg.dataGrp);
  uartPuts(", rfNode = ");
  putUint(config.baseCfg.nodeID);
  uartPuts(", rfPower = ");
  putUint(config.dataTxCfg.rfmPwr);
  uartPuts(", rfFormat = LowPowerLabs\r\n");
}

static void printSettingRFFreq(void) {
  switch (config.dataTxCfg.rfmFreq) {
  case 0:
    uartPuts("868");
    break;
  case 1:
    uartPuts("915");
    break;
  case 2:
    uartPuts("433.00");
    break;
  case 3:
    uartPuts("433.92");
    break;
  }
}

static void printSettingUART(void) {
  uartPuts("serial = ");
  uartPuts((config.dataTxCfg.txType & 0x2u) ? "on" : "off");
  uartPuts("\r\n");
}

static void printSettings(void) {
  if (1u != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return;
  }

  if ((cmdArgs.argv[0][1] != '\0') &&
      !((cmdArgs.argv[0][1] == 'h') && (cmdArgs.argv[0][2] == '\0'))) {
    uartPutsError("unknown command");
    return;
  }

  if ('h' == cmdArgs.argv[0][1]) {
    printSettingsHR();
  } else {
    printSettingsKV();
  }

  if (unsavedChange) {
    uartPuts("There are unsaved changes. Command \"s\" to save.\r\n\r\n");
  } else {
    uartPuts("All settings saved.\r\n\r\n");
  }
}

static void printSettingsHR(void) {
  uartPuts("\r\n\r\n==== Settings ====\r\n\r\n");

  uartPuts("Base Node ID      : ");
  putUint(config.baseCfg.nodeID);
  uartPuts("\r\n");

  uartPuts("Report time (s)   : ");
  putUint(config.baseCfg.reportTime);
  uartPuts("\r\n");

  uartPuts("OneWire interface : ");
  if (config.baseCfg.extTempEn) {
    uartPuts("En");
  } else {
    uartPuts("Dis");
  }
  uartPuts("abled\r\n");

  uartPuts("Data transmission :\r\n");
  if (config.dataTxCfg.txType & 0x1) {
    uartPuts("  - RFM69 (LowPowerLabs), ");
    printSettingRFFreq();
    uartPuts(" MHz @ ");
    putUint(config.dataTxCfg.rfmPwr - 18u);
    uartPuts("dBm\r\n");
  }
  if (config.dataTxCfg.txType & 0x2) {
    uartPuts("  - Serial enabled");
    if (config.baseCfg.useJson) {
      uartPuts(" (JSON)");
    }
    uartPuts("\r\n");
  }

  uartPuts("Pulse channel     : ");
  if (config.pulseCfg.active) {
    const uint8_t pu = config.pulseCfg.pu;
    uartPuts("Enabled\r\n  - Hysteresis (ms): ");
    putUint(config.pulseCfg.timeMask);
    uartPuts("\r\n");
    uartPuts("  - Pull :");
    uartPuts((0 == pu) ? "off" : ((1 == pu) ? "down" : "up"));
  } else {
    uartPuts("Disabled");
  }
  uartPuts("\r\n\r\n");
}

static void printSettingsKV(void) {
  printSettingPeriod();
  printSettingRF();
  printSettingPulse();
  printSettingUART();
  printSettingJSON();
  printSettingCO2();
}

static void putUint(const uint32_t u) {
  char strBuffer[12];
  (void)utilUtoa(strBuffer, u, ITOA_BASE10);
  uartPuts(strBuffer);
}

static void putUniqueID(void) {
  char strBuffer[8];
  for (size_t i = 0; i < 4u; i++) {
    utilUtoa(strBuffer, getUniqueID(i), ITOA_BASE16);
    uartPuts(strBuffer);
  }
}

static bool requireExactArgs(const size_t n) {
  if (n != cmdArgs.argc) {
    uartPutsError("unexpected arguments");
    return false;
  }

  return true;
}

static void uartPutsError(const char *msg) {
  uartPuts("> ERROR: ");
  uartPuts(msg);
  uartPuts("\r\n");
}

void configCmdChar(const uint8_t c) {
  if (('\r' == c) || ('\n' == c)) {
    if (!cmdPending) {
      uartPuts("\r\n");
      cmdPending = true;
    }
  } else if (('\b' == c)) {
    uartPuts("\b \b");
    if (0 != inBufferIdx) {
      inBufferIdx--;
      inBufferVolatile[inBufferIdx] = 0;
    }
  } else if ((inBufferIdx < (IN_BUFFER_W - 1)) && utilCharPrintable(c)) {
    inBufferVolatile[inBufferIdx++] = c;
  } else {
    inBufferClear();
    uartPuts("\r\n");
  }
}

void configEnter(void) {
  portPinDrv(PIN_LED, PIN_DRV_SET);
  inBufferClear();

  uartPuts("\033c==== emonTH3 Configuration ====\r\n\r\n");
  uartPuts("'?' to list commands\r\n\r\n");
  while (1) {
    if (cmdPending) {
      cmdPending = false;
      if (configProcessCmd()) {
        break;
      };
    }
    samlSleepEnter();
  }
  if (unsavedChange) {
    uartPuts("> Unsaved changes not written.\r\n");
  }

  uartPuts("\r\n====== End Configuration ======\r\n\r\n");
  portPinDrv(PIN_LED, PIN_DRV_CLR);
}

void configFirmwareBoardInfo(void) {
  struct EmonTHBuildInfo buildInfo = emonTH_build_info();

  uartPuts("\033c==== emonTH3 ====\r\n\r\n");

  uartPuts("> Board:\r\n");
  uartPuts("  - emonTH3\r\n");
  uartPuts("  - Serial:     ");
  putUniqueID();
  uartPuts("\r\n  - Last reset: ");
  uartPuts(getLastReset());
  uartPuts("\r\n");

  uartPuts("> Firmware:\r\n");
  uartPuts("  - Version:    ");
  uartPuts(buildInfo.release);
  uartPuts("\r\n");
  uartPuts("  - Build:      ");
  uartPuts(emonTH_build_info_string());
  uartPuts("\r\n\r\n");
  uartPuts("  - Distributed under GPL3 license, see COPYING.md\r\n");
  uartPuts("  - emonTH Copyright (C) 2024-26 Angus Logan\r\n");
  uartPuts("  - For Bear and Moose\r\n\r\n");
}

EmonTHConfigPacked_t *configLoadFromNVM(void) {
  EmonTHConfigPacked_t *pCfg = (EmonTHConfigPacked_t *)nvmPageBuffer();

  NVMStatus_t nvm = nvmDataFlashRead(NVM_PAGE_CONFIG);

  if (NVM_READ_OK != nvm) {
    configDefault();
    nvmPageBufferClear();
    memcpy(pCfg, &config, sizeof(config));
    nvmDataFlashWrite(NVM_PAGE_CONFIG, sizeof(config));
  }

  memcpy(&config, pCfg, sizeof(*pCfg));
  memcpy(&configNVM, pCfg, sizeof(*pCfg));
  return &config;
}

static bool configProcessCmd(void) {
  bool         exitConfig = false;
  unsigned int arglen     = 0;
  bool         termFound  = false;

  /* Help text - serves as documentation interally as well */
  static const char helpText[] =
      "\r\n"
      "emonTH information and configuration commands\r\n\r\n"
      " - ?             : show this text again\r\n"
      " - a <a> <t>     : Configure SCD4x and STCC-4 CO2 sensors\r\n"
      "     -  a : altitude above sea level (m) (s)\r\n"
      "     -  t : sample interval (s); SCD4x only.\r\n"
      " - c<n>          : enable UART. n = 0: OFF, n = 1: ON\r\n"
      " - d<n>          : set the data acquisition period\r\n"
      " - e<n>          : number of external temperature sensors (0, 1, or "
      "4)\r\n"
      " - f             : exit, lock, and continue\r\n"
      " - j<n>          : JSON serial format. n = 0: OFF, n = 1: ON\r\n"
      " - l             : list settings (key / value)\r\n"
      " - lh            : list settings (human readable)\r\n"
      " - m <x> <y> <z> : Pulse counting\r\n"
      "     - x = 0: OFF, x = 1, ON\r\n"
      "     - y = n: no pull, y = d : pull down, y = u : pull up. Only for x = "
      "1\r\n"
      "     - z : minimum period (ms). Only for x = 1\r\n"
      " - n<n>          : set node ID [1..60].\r\n"
      " - p<n>          : set the RF power level\r\n"
      " - r[s]          : restore defaults, rs to restore saved config\r\n"
      " - s             : save settings to NVM\r\n"
      " - v             : firmware and board information\r\n"
      " - w<n>          : enable wireless. n = 0: OFF, n = 1: ON\r\n"
      " - x<n>          : 433 MHz compatibility. n = 0: 433.92 MHz, n = 1: "
      "433.00 MHz\r\n";

  /* Copy volatile input buffer into command buffer */
  for (size_t i = 0; i < IN_BUFFER_W; i++) {
    inBuffer[i] = inBufferVolatile[i];
  }

  /* Convert \r or \n to 0, and get the length until then. */
  while (!termFound && (arglen < IN_BUFFER_W)) {
    if (0 == inBuffer[arglen]) {
      termFound = true;
      break;
    }
    arglen++;
  }

  if (!termFound) {
    return false;
  }

  cmdArgs = inBufferTok();

  if (0 == cmdArgs.argc) {
    inBufferClear();
    return false;
  }

  /* Decode on first character in the buffer */
  switch (cmdArgs.argv[0][0]) {
  case '?':
    if (requireExactArgs(1u)) {
      uartPuts(helpText);
    }
    break;
  case 'a':
    (void)configCO2();
    break;
  case 'c':
    (void)configUART();
    break;
  case 'd':
    (void)configDatalog();
    break;
  case 'e':
    (void)configExtTempMax();
    break;
  case 'f':
    if (requireExactArgs(1u)) {
      exitConfig = true;
    }
    break;
  case 'j':
    (void)configJSON();
    break;
  case 'l':
    printSettings();
    break;
  case 'm':
    (void)configPulse();
    break;
  case 'n':
    (void)configNodeID();
    break;
  case 'p':
    (void)configRFPower();
    break;
  case 'r':
    configRestore();
    break;
  case 's':
    if (requireExactArgs(1u)) {
      configSaveToNVM();
    }
    break;
  case 'v':
    if (requireExactArgs(1u)) {
      configFirmwareBoardInfo();
    }
    break;
  case 'w':
    (void)configRFM();
    break;
  case 'x':
    (void)configRF433();
    break;
  default:
    uartPutsError("unknown command");
    break;
  }

  unsavedChange = configCheckUnsaved();

  cmdPending = false;
  inBufferClear();
  return exitConfig;
}

void configSaveToNVM(void) {
  if (unsavedChange) {
    nvmPageBufferClear();
    memcpy(nvmPageBuffer(), &config, sizeof(config));
    nvmDataFlashWrite(NVM_PAGE_CONFIG, sizeof(config));

    unsavedChange = false;
    memcpy(&configNVM, &config, sizeof(config));
    uartPuts("> All settings saved.\r\n");
  } else {
    uartPuts("> No changes to save.\r\n");
  }
}

/* =======================
 * UART Interrupt handler
 * ======================= */

void SERCOM_UART_HANDLER_RXC {
  /* Echo the received character to the TX channel, and send to the command
   * stream.
   */

  emonTHInteractiveUartSet();
  if (uartGetcReady()) {
    uint8_t rx_char = uartGetc();
    configCmdChar(rx_char);

    if (utilCharPrintable(rx_char) && !cmdPending) {
      uartPutcBlocking(rx_char);
    }
  }
}
