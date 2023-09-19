/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 17-july-2023
 *      Author: niles
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

// ********1. configuration structure of SPI
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


// ********2. handle structure of SPI
typedef struct{
	SPI_RegDef_t   *pSPIx;
	SPI_Config_t    SPI_Config;
}SPI_Handle_t;

//********* 3. macros related to SPI Registers

// @spi device mode
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0

// @spi bus config
#define SPI_BUS_CONFIG_FD              1
#define SPI_BUS_CONFIG_HD              2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY  3

// @SPI SclkSpeed
#define SPI_SCLK_SPEED_DIV2   0
#define SPI_SCLK_SPEED_DIV4   1
#define SPI_SCLK_SPEED_DIV8   2
#define SPI_SCLK_SPEED_DIV16  3
#define SPI_SCLK_SPEED_DIV32  4
#define SPI_SCLK_SPEED_DIV64  5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV265 7

// @spi DFF
#define SPI_DFF_8BITS   0
#define SPI_DFF_16BITS  1

// @CPOL
#define SPI_CPOL_HIGH  1
#define SPI_CPOL_LOW   0

// @CPHA
#define SPI_CPHA_HIGH  1
#define SPI_CPHA_LOW   0

// @ SPI SSM
#define SPI_SSM_EN   1
#define SPI_SSM_DI   0

// spi related status flages definations
#define SPI_TXE_FLAG   (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG  (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG  (1 << SPI_SR_BUSY)



//*******************************************************************************************************************
//**                         4. functions for APIs supported by the drivers                                        **
//**               for mor information about the APIs check the function defination                                **
//*******************************************************************************************************************

// 4.1 peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// 4.2 Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// 4.3 Data Send and Recive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void _ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// 4.4 SPIIRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// 4.5 other peripheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
