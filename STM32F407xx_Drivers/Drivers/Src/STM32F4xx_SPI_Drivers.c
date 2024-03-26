/*
 * STM32F407xx_SPI_Drivers.c
 *
 *  Created on: 27-Oct-2023
 *      Author: niles
 */

#include "STM32F407xx_SPI_Drivers.h"

// APIs implemenration through function

/////////////// 1. peripheral clock setup (1 function) //////////////

/*********************************************************
  ----Function documentation--------------------------
  *@ Function  - SPI_PeriClockControl
  *@ Brief     - this function enable or disable peripheral clock for the given SPI port
  *@ param[in] - Base address of the SPI peripheral
  *@ param[in] - ENABLE or DISABLE macros
  *@ return    - void
  *@ note      - Using Else if cheaking
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

///////////// 2. init and De-init (2 function) /////////////////

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - SPI_Initialization
  *@ Brief     -
  *@ param[in] - Handle structure --> pSPIx, SPI_Config --> structure
  *@ return    - void
  *@ note      - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 	first configureb the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleard
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. configure the spi serial clock speed (baudrate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DDf
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*****************************************************
 ----Function documentation--------------------------
  *@ Function  - SPI_DeInit
  *@ Brief     - reset all the register of peripherals
  *@ param[in] - SPI name
  *@ return    - void
  *@ note      - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
			SPI1_REG_RESET();
		}else if(pSPIx == SPI2){
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_REG_RESET();
		}
}


//////////////// 3. data SEND AND RECIve (2 functions)  /////////////
/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - Send Data
  *@ Brief     -
  *@ param[in] - base addr of SPI
  *@ param[in] - pointer to data
  *@ param[in] - number of byte transmited
  *@ return    -
  *@ note      - this is the blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - Recive Data
  *@ Brief     -
  *@ param[in] - Gpio port name
  *@ param[in] - Gpio pin number
  *@ return    - value of pin number 0/1
  *@ note      - none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data FROM DR to rx buffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


//////////////// 5. Enable to SPI peripheral (4 functions)  /////////////
/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - SPi peripheral control
  *@ Brief     -
  *@ param[in] - base addr of SPI
  *@ param[in] -  Enable or disable
  *@ return    -
  *@ note      -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - SPi ssi config
  *@ Brief     -
  *@ param[in] - base addr of SPI
  *@ param[in] -  Enable or disable
  *@ return    -
  *@ note      -
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - SPi ssoe config
  *@ Brief     -
  *@ param[in] - base addr of SPI
  *@ param[in] -  Enable or disable
  *@ return    -
  *@ note      -
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - To get flag status
  *@ Brief     -
  *@ param[in] - base addr of SPI
  *@ param[in] -  Enable or disable
  *@ return    -
  *@ note      -
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


// 4.4 SPIIRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


