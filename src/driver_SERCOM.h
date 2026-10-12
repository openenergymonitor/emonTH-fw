#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver_PORT.h"

typedef enum I2CM_Ack_ { I2CM_ACK = 0u, I2CM_NACK = 1u } I2CM_Ack_t;

typedef enum I2CM_AckCmd_ {
  I2CM_ACK_CMD_NONE     = 0u,
  I2CM_ACK_CMD_START    = 1u,
  I2CM_ACK_CMD_CONTINUE = 2u,
  I2CM_ACK_CMD_STOP     = 3u
} I2CM_AckCmd_t;

typedef enum I2CM_Status_ {
  I2CM_SUCCESS,
  I2CM_ERROR,
  I2CM_TIMEOUT,
  I2CM_NOACK
} I2CM_Status_t;

/*! @brief Configure the serial communication modules */
void sercomSetup(void);

/*! @brief Send I2C address byte (address + R/W bit) and wait for response.
 *  @param [in] addr : 8-bit address byte (7-bit address << 1 | R/W bit)
 *  @return I2C status (success, timeout, or NACK)
 */
I2CM_Status_t i2cActivate(const uint8_t addr);

/*! @brief Issue I2C ACK/NACK and optional bus command.
 *  @param [in] ack : 0: ACK, 1: NACK
 *  @param [in] cmd : ACK command (none/continue/stop)
 */
void i2cAck(I2CM_Ack_t ack, I2CM_AckCmd_t cmd);

/*! @brief Write a data byte to the I2C bus (blocking).
 *  @param [in] data : data byte
 */
void i2cDataWrite(const uint8_t data);

/*! @brief Read a data byte from the I2C bus (blocking).
 *  @return read data
 */
uint8_t i2cDataRead(void);

/*! @brief Disable the I2C interface */
void i2cDisable(void);

/*! @brief Enable the I2C interface */
void i2cEnable(void);

/*! @brief Indicate if the I2C interface is active
 *  @return true if enabled, false otherwise
 */
bool i2cEnabled(void);

/*! @brief Select an SPI peripheral
 *  @param [in] nSS : grp+pin of chip select line
 */
void spiDeSelect(const Pin_t nSS);

/*! @brief Disable the SPI peripheral */
void spiDisable(void);

/*! @brief Enable the SPI peripheral */
void spiEnable(void);

/*! @brief Select an SPI peripheral
 *  @param [in] nSS : grp+pin of chip select line
 */
void spiSelect(const Pin_t nSS);

/*! @brief Send a buffer on the configured SPI channel (blocking).
 *  @param [in] pSrc : pointer to the source buffer
 *  @param [in] n : number of bytes to send
 */
void spiSendBuffer(const void *pSrc, size_t n);

/*! @brief Send a byte on the configured SPI channel
 *  @param [in] b : byte to send
 *  @return data in the SPI Rx buffer
 */
uint8_t spiSendByte(const uint8_t b);

/*! @brief Disable the UART entirely, setting pins as pulled-up inputs */
void uartDisable(void);

/*! @brief Get a character from the USART data buffer.
 *  @return character in buffer
 */
char uartGetc(void);

/*! @brief Indicate if a byte is waiting in the USART data buffer.
 *  @return true if waiting, false otherwise
 */
bool uartGetcReady(void);

/*! @brief Send a single character (blocking) on UART
 *  @param [in] c : Single character
 */
void uartPutcBlocking(const char c);

/*! @brief Send a string (blocking) on UART
 *  @param [in] s : Pointer to null terminated string
 */
void uartPutsBlocking(const char *s);

/*! @brief Send a string (non-blocking) on UART via DMA.
 *         The source buffer must remain valid until dmacUARTComplete() is true.
 *  @param [in] s : pointer to the string
 *  @param [in] len : length of the string (not including NULL)
 */
void uartPutsNonBlocking(const char *const s, uint32_t len);

/*! @brief Disable the UART's Rx channel and interrupt */
void uartRxDisable(void);

/*! @brief Enable the UART's Rx channel and interrupt */
void uartRxEnable(void);
