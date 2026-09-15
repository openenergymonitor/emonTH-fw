#include <string.h>

#include "driver_DMAC.h"
#include "driver_NVM.h"
#include "emonTH_assert.h"
#include "emonTH_saml.h"

/* Configuration key - indicates that the configuration is the default or
 * has been retrieved NVM. */
#define CONFIG_NVM_KEY 0xca55e77eul

typedef struct __attribute__((__packed__)) NVMHeader_ {
  uint32_t watermark;  /* Indicate the page is in use */
  uint16_t crc16;      /* CRC16 CCITT */
  uint8_t  writeCount; /* Number of times this page has been written */
  uint8_t  n;          /* Number of data bytes in page */
} NVMHeader_t;

/* The NVM page buffer must be 4 byte aligned for allow access from DFLASH */
uint8_t pageBuffer[FLASH_PAGE_SIZE] __attribute__((aligned(16))) = {0};

NVMStatus_t nvmDataFlashRead(const NVMPage_t page) {
  EMONTH_ASSERT(page < NVMCTRL_DATAFLASH_PAGES);

  NVMStatus_t status = NVM_READ_OK;

  uint32_t *pDst = (uint32_t *)pageBuffer;

  const volatile uint32_t *addr =
      (const volatile uint32_t *)((FLASH_PAGE_SIZE * page) + NVMCTRL_DATAFLASH);

  for (size_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pDst)); i++) {
    *pDst++ = *addr++;
  }

  NVMHeader_t *header = (NVMHeader_t *)pageBuffer;

  if (CONFIG_NVM_KEY != header->watermark) {
    return NVM_READ_NO_INIT;
  }

  if (header->n > (FLASH_PAGE_SIZE - sizeof(*header))) {
    return NVM_READ_BAD_CRC;
  }

  if (calcCRC16_ccitt(pageBuffer + sizeof(*header), header->n) !=
      header->crc16) {
    return NVM_READ_BAD_CRC;
  }

  return status;
}

void nvmDataFlashWrite(const NVMPage_t page, const size_t n) {
  const uint32_t    *pBuf       = (const uint32_t *)pageBuffer;
  volatile uint32_t *nvmAddress = (volatile uint32_t *)NVMCTRL_DATAFLASH;

  NVMHeader_t *header = (NVMHeader_t *)pageBuffer;

  if (CONFIG_NVM_KEY != header->watermark) {
    header->watermark  = CONFIG_NVM_KEY;
    header->writeCount = 1;
  } else {
    header->writeCount++;
  }
  header->crc16 = calcCRC16_ccitt(pageBuffer + sizeof(*header), n);
  header->n     = n;

  /* Flush anything outstanding in the page buffer */
  if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_LOAD) {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
  }
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;

  /* Set the correct address in DFLASH region */
  const uint16_t aoffset = page * FLASH_PAGE_SIZE;
  NVMCTRL->ADDR.reg =
      NVMCTRL_ADDR_ARRAY_DATAFLASH | NVMCTRL_ADDR_AOFFSET(aoffset);

  /* Delete the row */
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;

  /* Write to the page buffer, and write out complete page when done */
  for (size_t i = 0; i < (FLASH_PAGE_SIZE / sizeof(*pBuf)); i++) {
    *nvmAddress++ = *pBuf++;
  }

  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
  while (!(NVMCTRL->STATUS.reg & NVMCTRL_STATUS_READY))
    ;
}

uint8_t *nvmPageBuffer(void) { return pageBuffer + sizeof(NVMHeader_t); }

void nvmPageBufferClear(void) {
  memset((pageBuffer + sizeof(NVMHeader_t)), 0,
         (sizeof(pageBuffer) - sizeof(NVMHeader_t)));
}
