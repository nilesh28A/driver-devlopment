/*
 * STM32F4xx_SPI_Drivers.h
 *
 *  Created on: 22-Jul-2023
 *      Author: niles
 */

#ifndef INC_STM32F4XX_SPI_DRIVERS_H_
#define INC_STM32F4XX_SPI_DRIVERS_H_

#include "STM32F4xx.h"

// 9. configuration structure of SPI peripherals
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


// 10. handle structure of GPIO
typedef struct{

	//pointer to hold the base address of the GPIO peripherals
	SPI_RegDef_t      *pSPIx;
	SPI_PinConfig_t    SPIConfig;
}SPI_Handle_t;


//******************11. functions for APIs supported by the drivers ******************************************

// 11.1 peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *SPIx, uint8_t EnorDi);

// 11.2 Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//  DATA send and receive
void SPI_SendData(SPI_RegDef_t *SPIx, uin8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPI_RegDef_t *SPIx, uin8_t *pRxBuffer, uint32_t Len);

//  IRQ Configuration and ISR Handling
void SPI_IRQIntrruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQProrityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handal_t *pHandle);




#endif /* INC_STM32F4XX_SPI_DRIVERS_H_ */
