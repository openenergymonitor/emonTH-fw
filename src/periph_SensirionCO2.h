#pragma once

/*! @brief Discover any SCD4x devices
 *  @param [in] altitude : height above sea level (m)
 */
void scd4xDiscover(const uint16_t altitude);

/*! @brief Measure CO2 level
 *  @return Measured CO2 level (ppm)
 */
uint16_t scd4xMeasureCO2(void);

/*! @brief Indicate if there is an SCD4x present
 *  @return true if present, false otherwise
 */
bool scd4xPresent(void);

/*! @brief Discover any STCC-4 devices
 *  @param [in] altitude : height above sea level (m)
 *  @param [in] reset : factory reset
 */
void stcc4Discover(const uint16_t altitude, const bool reset);

/*! @brief Measure CO2 level
 *  @return Measured CO2 level (ppm). If there is an error, 0 ppm is returned.
 */
uint16_t stcc4MeasureCO2(void);

/*! @brief Indicate if there is an STCC-4 present
 *  @return true if present, false otherwise
 */
bool stcc4Present(void);

/*! @brief Set the STCC-4's RH-T compensation
 *  @param [in] t : temperature (°C)
 *  @param [in] rh : relative humidity (%)
 */
void stcc4SetRHT(const int16_t t, const uint16_t rh);
