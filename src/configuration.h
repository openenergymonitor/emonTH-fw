#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board_def.h"

/* Configurable options. All the structs are packed to allow simple write to
 * EEPROM as a contiguous set.
 */
typedef enum DataTx_ {
  DATATX_RFM69 = 1u,
  DATATX_UART  = 2u,
  DATATX_BOTH  = 3u
} TxType_t;

typedef struct __attribute__((__packed__)) BaseCfg_ {
  uint8_t  nodeID;     /* ID for report*/
  uint8_t  dataGrp;    /* Transmission group - default 210 */
  bool     useJson;    /* JSON format for serial output */
  uint8_t  extTempEn;  /* Number of external temperature sensor enabled */
  uint16_t reportTime; /* Time between reports (s) */
  uint8_t  res0[2];
} BaseCfg_t;

typedef struct __attribute__((__packed__)) DataTxCfg_ {
  uint8_t txType;  /* UART, RFM on SPI, or both */
  uint8_t rfmFreq; /* 0: 868, 1: 915, 2: 433.00, 3: 433.92 (MHz) */
  uint8_t rfmPwr;
  uint8_t res0;
} DataTxCfg_t;

typedef struct __attribute__((__packed__)) PulseCfgPacked_ {
  uint8_t timeMask; /* Minimum time between pulses */
  bool    active;   /* In use */
  uint8_t pu;       /* Pull config; 0 : none, 1 : down, 2 : up */
  uint8_t res0;
} PulseCfgPacked_t;

typedef struct __attribute__((__packed__)) SCDCfgPacked_ {
  uint16_t altitude;       /* Height above sea level */
  uint16_t sampleInterval; /* Time between CO2 samples (s) */
  uint8_t  res0[4];
} SCDCfgPacked_t;

typedef struct __attribute__((__packed__)) EmonTHConfigPacked_ {
  BaseCfg_t        baseCfg;
  DataTxCfg_t      dataTxCfg;
  PulseCfgPacked_t pulseCfg;
  SCDCfgPacked_t   scdCfg;
} EmonTHConfigPacked_t;

_Static_assert(sizeof(EmonTHConfigPacked_t) < 57,
               "EmonTHConfigPacked_t bigger than 56 bytes");

/*! @brief Add a character to the command stream.
 *         Input is buffered until a command terminator is received.
 *  @param [in] c : character to add
 */
void configCmdChar(const uint8_t c);

/*! @brief Enter the configuration mode */
void configEnter(void);

/*! @brief Print the board and firmware information to serial */
void configFirmwareBoardInfo(void);

/*! @brief Load configuration from NVM, falling back to defaults.
 *  @return pointer to current configuration (static storage)
 */
EmonTHConfigPacked_t *configLoadFromNVM(void);

/*! @brief Get STCC-4 reset flag
 *  @return true for reset, false otherwise
 */
bool configSTCC4Reset(void);

/*! @brief Return one word from the SAML's unique ID
 *  @param [in] idx : index of the word to fetch
 *  @return word idx from the unique ID
 */
uint32_t getUniqueID(const size_t idx);
