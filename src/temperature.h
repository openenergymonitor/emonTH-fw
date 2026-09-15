#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum TEMP_INTF_ { TEMP_INTF_ONEWIRE, TEMP_INTF_I2C } TEMP_INTF_t;

typedef enum TempStatus_ {
  TEMP_OK,
  TEMP_OVERRUN,
  TEMP_NO_SENSORS,
  TEMP_FAILED,
  TEMP_NO_SAMPLE,
  TEMP_BAD_CRC,
  TEMP_BAD_SENSOR,
  TEMP_OUT_OF_RANGE
} TempStatus_t;

/* OneWire sample values are fixed-point in 1/16 deg C. These values are
 * outside the DS18B20 valid range and match the OEM sentinel encoding.
 */
#define TEMP_ONEWIRE_RAW_UNUSED       4800 /* 300 deg C */
#define TEMP_ONEWIRE_RAW_OUT_OF_RANGE 4832 /* 302 deg C */
#define TEMP_ONEWIRE_RAW_FAILED       4864 /* 304 deg C */

typedef struct TempRead_ {
  TempStatus_t status;
  int16_t      result;
} TempRead_t;

/*! @brief Find and initialise sensors.
 *  @param [in] intf : interface type
 *  @param [in] pParams : parameters for given interface type (NULL if unused)
 *  @return number of sensors found
 */
size_t tempSensorsInit(const TEMP_INTF_t intf, const void *pParams);

/*! @brief Read temperature samples from all monitors.
 *         For TEMP_INTF_ONEWIRE, values are fixed-point in 1/16 °C.
 *  @param [in] intf : interface type
 *  @param [out] pDst : pointer to output array, at least TEMP_MAX_ONEWIRE
 *                     entries for OneWire.
 */
TempStatus_t tempSampleRead(const TEMP_INTF_t intf, int16_t *pDst);

/*! @brief Start a temperature sample.
 *  @param [in] intf : interface type
 *  @param [in] dev : device index (ignored for OneWire global convert)
 */
TempStatus_t tempSampleStart(const TEMP_INTF_t intf, const size_t dev);
