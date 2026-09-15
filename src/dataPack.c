#include <stdbool.h>
#include <string.h>

#include "dataPack.h"
#include "emonTH_assert.h"
#include "periph_HDC2010.h"
#include "temperature.h"
#include "util.h"

#define CONV_STR_W (16u)

enum {
  STR_TEMPEX = 0,
  STR_BATT   = 1,
  STR_HUMID  = 2,
  STR_PERIOD = 3,
  STR_PULSE  = 4,
  STR_TEMP   = 5,
  STR_COLON  = 6,
  STR_CRLF   = 7,
  STR_DQUOTE = 8,
  STR_LCURL  = 9,
  STR_RCURL  = 10,
  STR_COMMA  = 11,
  STR_CO2    = 12
};

/* "Fat" string with current length and buffer size. */
typedef struct StrN {
  char  *str; /* Pointer to the string */
  size_t n;   /* Length of the string  */
  size_t m;   /* Buffer length */
} StrN_t;

static void   catId(StrN_t *strD, int id, const size_t field, const bool json);
static void   initFields(StrN_t *pD, char *pS, const size_t m);
static size_t strnCat(StrN_t *strD, const StrN_t *strS);
static size_t strnCatDeci(StrN_t *strD, const int32_t v);
static size_t strnCatInt(StrN_t *strD, const int32_t v);
static size_t strnCatUint(StrN_t *strD, const uint32_t v);

static char tmpStr[CONV_STR_W] = {0};

/* Strings that are inserted in the transmitted message */
const StrN_t baseStr[] = {
    {.str = "tempex", .n = 6, .m = 7},   {.str = "batt", .n = 4, .m = 5},
    {.str = "humidity", .n = 8, .m = 9}, {.str = ".", .n = 1, .m = 2},
    {.str = "pulse", .n = 5, .m = 6},    {.str = "temp", .n = 4, .m = 5},
    {.str = ":", .n = 1, .m = 2},        {.str = "\r\n", .n = 2, .m = 3},
    {.str = "\"", .n = 1, .m = 2},       {.str = "{", .n = 1, .m = 2},
    {.str = "}", .n = 1, .m = 2},        {.str = ",", .n = 1, .m = 2},
    {.str = "co2", .n = 3, .m = 4}};

/*! @brief Append "<field><id>:" to the string.
 *  @param [out] strD : pointer to the fat string
 *  @param [in] id : numeric index (-1 to omit index)
 *  @param [in] field : field name index, e.g. "STR_V"
 *  @param [in] json : output in JSON format
 */
static void catId(StrN_t *strD, int id, const size_t field, const bool json) {

  /* No comma for the 1st field */
  if (field != STR_TEMP) {
    strD->n += strnCat(strD, &baseStr[STR_COMMA]);
  }

  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[field]);

  if (id > -1) {
    strD->n += strnCatInt(strD, id);
  }
  if (json) {
    strD->n += strnCat(strD, &baseStr[STR_DQUOTE]);
  }
  strD->n += strnCat(strD, &baseStr[STR_COLON]);
}

static void initFields(StrN_t *pD, char *pS, const size_t m) {
  /* Setup destination string */
  pD->str = pS;
  pD->n   = 0;
  pD->m   = m;
  memset(pD->str, 0, m);
}

static size_t strnCatFromTmp(StrN_t *strD, const size_t len) {
  size_t toCopy = 0;

  if (strD->n < strD->m) {
    const size_t space = strD->m - strD->n;
    toCopy             = (len < space) ? len : space;
  }

  if (toCopy) {
    memcpy(strD->str + strD->n, tmpStr, toCopy);
  }

  return len;
}

static size_t strnCatInt(StrN_t *strD, const int32_t v) {
  return strnCatFromTmp(strD, utilItoa(tmpStr, v, ITOA_BASE10) - 1u);
}

static size_t strnCatUint(StrN_t *strD, const uint32_t v) {
  return strnCatFromTmp(strD, utilUtoa(tmpStr, v, ITOA_BASE10) - 1u);
}

static size_t strnCat(StrN_t *strD, const StrN_t *strS) {
  size_t bytesToCopy = 0;

  if (strD->n < strD->m) {
    const size_t space = strD->m - strD->n;
    bytesToCopy        = (strS->n < space) ? strS->n : space;
  }

  if (bytesToCopy) {
    memcpy((strD->str + strD->n), strS->str, bytesToCopy);
  }

  return strS->n;
}

static size_t strnCatDeci(StrN_t *strD, const int32_t v) {
  uint32_t mag = (uint32_t)v;
  size_t   n   = 0;
  size_t   appended;
  StrN_t   str = *strD;

  if (v < 0) {
    mag      = (uint32_t)(-(v + 1)) + 1u;
    appended = strnCat(&str, &(StrN_t){.str = "-", .n = 1, .m = 2});
    n += appended;
    str.n += appended;
  }

  appended = strnCatUint(&str, mag / 10u);
  n += appended;
  str.n += appended;
  appended = strnCat(&str, &baseStr[STR_PERIOD]);
  n += appended;
  str.n += appended;
  appended = strnCatUint(&str, mag % 10u);
  n += appended;
  return n;
}

void dataPackPacked(const EmonTHDataset_t *restrict pData,
                    void *restrict pPacked) {

  const int16_t  tInt = hdc2010ConvTx10(pData->hdcResRaw.temp);
  const uint16_t hInt = hdc2010ConvRHx10(pData->hdcResRaw.humidity);
  const uint16_t bInt = (uint16_t)((pData->battery * 3226u) / 10000u);

  /* T/H 10x value, e.g. 261 = 26.1ºC */
  if (4 == pData->numExtMax) {
    PackedData_4Ext_t *tx = (PackedData_4Ext_t *)pPacked;
    tx->tempInternal      = tInt;
    tx->humidityInternal  = hInt;
    tx->battery           = bInt;
    tx->pulse             = pData->pulseCnt;
    for (int i = 0; i < TEMP_MAX_ONEWIRE; i++) {
      tx->tempExternal[i] = (int16_t)((pData->tempExternal[i] * 6) +
                                      (pData->tempExternal[i] >> 2));
    }
    tx->co2 = pData->co2;
  } else {
    PackedData_1Ext_t *tx = (PackedData_1Ext_t *)pPacked;
    tx->tempInternal      = tInt;
    tx->humidityInternal  = hInt;
    tx->battery           = bInt;
    tx->pulse             = pData->pulseCnt;
    tx->tempExternal =
        (int16_t)((pData->tempExternal[0] * 6) + (pData->tempExternal[0] >> 2));
    tx->co2 = pData->co2;
  }
}

size_t dataPackSerial(const EmonTHDataset_t *restrict pData,
                      char *restrict pDst, const size_t m, const bool json) {
  EMONTH_ASSERT(pData);
  EMONTH_ASSERT(pDst);

  uint32_t     battery = pData->battery * 3226;
  int          tempInt = hdc2010ConvTx10(pData->hdcResRaw.temp);
  unsigned int humInt  = hdc2010ConvRHx10(pData->hdcResRaw.humidity);

  StrN_t strn;
  initFields(&strn, pDst, m);

  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_LCURL]);
  }

  catId(&strn, -1, STR_TEMP, json);
  strn.n += strnCatDeci(&strn, tempInt);

  for (int i = 0; i < (int)pData->numExtMax; i++) {
    /* Only include sensors that have been found */
    if (pData->tempExternal[i] != TEMP_ONEWIRE_RAW_UNUSED) {
      tempInt = pData->tempExternal[i] * 62500; /* micro-degrees */
      tempInt = tempInt / 100000;               /* deci-degrees */
      catId(&strn, (i + 1), STR_TEMPEX, json);

      strn.n += strnCatDeci(&strn, tempInt);
    }
  }

  catId(&strn, -1, STR_HUMID, json);
  strn.n += strnCatUint(&strn, (humInt / 10u));
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  strn.n += strnCatUint(&strn, (humInt % 10u));

  catId(&strn, -1, STR_BATT, json);
  strn.n += strnCatUint(&strn, (battery / 1000000u));
  strn.n += strnCat(&strn, &baseStr[STR_PERIOD]);
  strn.n += strnCatUint(&strn, ((battery % 1000000u) / 1000u));

  catId(&strn, -1, STR_PULSE, json);
  strn.n += strnCatUint(&strn, pData->pulseCnt);

  catId(&strn, -1, STR_CO2, json);
  strn.n += strnCatUint(&strn, pData->co2);

  /* Terminate with } for JSON and \r\n */
  if (json) {
    strn.n += strnCat(&strn, &baseStr[STR_RCURL]);
  }
  strn.n += strnCat(&strn, &baseStr[STR_CRLF]);
  return strn.n;
}
