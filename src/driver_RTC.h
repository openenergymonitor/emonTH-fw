#pragma once

#include <stdint.h>

#include "emonTH.h"

#define RTC_PERIOD_MIN_SECONDS 1u
#define RTC_PERIOD_MAX_SECONDS 16384u

typedef struct RTC_Evt_ {
  uint16_t smpInterval; /* Sample interval (s) */
  EVTSRC_t evt;         /* Event ID */
} RTC_Evt_t;

/*! @brief Enable the RTC counter with interrupt on overflow.
 *         Requires rtcSetup() to have been called.
 *  @param [in] period : RTC overflow period (s)
 */
void rtcEnable(const uint16_t period);

/*! @brief Register a periodic event with the RTC.
 *  @param [in] rtcevt : event to be registered (interval in seconds)
 */
void rtcEvtReg(const RTC_Evt_t rtcevt);

/*! @brief Setup the RTC module */
void rtcSetup(void);
