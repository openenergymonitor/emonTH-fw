#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board_def.h"
#include "periph_HDC2010.h"

_Static_assert((sizeof(bool) == 1), "bool must be 1 byte");

#define WAKE_PERIOD_DEF 55u

/*********************************
 * Device configuration
 *********************************/

#define NETWORK_GROUP_DEF 210u /* Must match emonBase group */
#define NODE_ID_DEF       27u  /* Default node ID */
#define RFM_AES_DEF       "89txbe4p8aik5kt3"
#define TEMP_NUM_DEF      1u

typedef struct EmonTHCfg_ {
  uint8_t RF_Freq;
  uint8_t networkGroup;
  uint8_t nodeID;
  bool    idFromNVM;
  int8_t  txType;
  uint8_t rfPower;
  bool    pulseEnabled;
  uint8_t pulsePeriod;
  uint8_t extTempEnabled;
} EmonTHCfg_t;

_Static_assert(sizeof(EmonTHCfg_t) < 57, "EmonThCfg_t bigger than 56 bytes");

/*********************************
 * Remaining
 *********************************/

#define TX_BUFFER_W 196u

typedef struct EmonTHDataset_ {
  HDCResultRaw_t hdcResRaw;
  int16_t        tempExternal[TEMP_MAX_ONEWIRE];
  uint32_t       battery;
  uint32_t       pulseCnt;
  size_t         numExtMax;
  uint16_t       co2;
} EmonTHDataset_t;

/* This struct must match the OEM definitions found at:
 * https://docs.openenergymonitor.org/electricity-monitoring/networking/sending-data-between-nodes-rfm.html
 */
typedef struct __attribute__((__packed__)) PackedData_4Ext_ {
  int16_t  tempInternal;
  int16_t  tempExternal[TEMP_MAX_ONEWIRE];
  uint16_t humidityInternal;
  uint16_t battery;
  uint32_t pulse;
  uint16_t co2;
} PackedData_4Ext_t;

typedef struct __attribute__((__packed__)) PackedData_1Ext_ {
  int16_t  tempInternal;
  int16_t  tempExternal;
  uint16_t humidityInternal;
  uint16_t battery;
  uint32_t pulse;
  uint16_t co2;
} PackedData_1Ext_t;

/* Maximum size of RFM69CW buffer is 61 bytes. Node, number, and CRC included.
 */
_Static_assert((sizeof(PackedData_4Ext_t) + 4) < 62,
               "PackedData_4Ext_t > 62 bytes");

/* EVTSRC_t contains all the event/interrupts sources. This value is shifted
 * to provide a vector of set events as bits.
 */
typedef enum EVTSRC_ {
  EVT_WAKE_TIMER,
  EVT_SCD4x_SAMPLE,
  EVT_LED_FLASH
} EVTSRC_t;

/*! @brief Clear a pending event/interrupt flag after the task has been handled.
 *  @param [in] evt : event source in enum
 */
void emonTHEventClr(const EVTSRC_t evt);

/*! @brief Set the pending event/interrupt flag for tasks not handled in ISR.
 *  @param [in] evt : event source in enum
 */
void emonTHEventSet(const EVTSRC_t evt);

/*! @brief Atomically test and clear a pending event flag.
 *  @param [in] evt : event source in enum
 *  @return true if pending, false otherwise
 */
bool emonTHEventTake(const EVTSRC_t evt);

/*! @brief Set the flag that any character has been received on UART */
void emonTHInteractiveUartSet(void);

/*! @brief Blocking write of a string to UART.
 *  @param [in] s : pointer to null terminated string
 */
void uartPuts(const char *s);
