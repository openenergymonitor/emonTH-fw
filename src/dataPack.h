#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "emonTH.h"

/*! @brief Pack the sample data into a packed structure for RFM transmission.
 *         Output type is PackedData_1Ext_t or PackedData_4Ext_t depending on
 *         pData->numExtMax.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pPacked : pointer to the destination packet
 */
void dataPackPacked(const EmonTHDataset_t *restrict pData,
                    void *restrict pPacked);

/*! @brief Packs the data packet into serial format.
 *         Returns the number of characters that would have been packed,
 *         regardless of the value of m. If the return value > m, then the
 *         buffer was truncated (similar to snprintf). Does not append
 *         a NULL. Clears data buffer in advance.
 *  @param [in] pData : pointer to the raw data
 *  @param [out] pDst : pointer to the destination buffer
 *  @param [in] m : width of the destination buffer
 *  @param [in] json : select JSON output (true), or K:V (false)
 *  @return the number of the characters that would be packed
 */
size_t dataPackSerial(const EmonTHDataset_t *restrict pData,
                      char *restrict pDst, const size_t m, const bool json);
