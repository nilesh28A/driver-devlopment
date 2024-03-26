/*
 * STM32F407xx_I2C_Drivers.c
 *
 *  Created on: 28-Oct-2023
 *      Author: niles
 */
#include "STM32F407xx_I2C_Drivers.h"

uint32_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

// APIs implemenration through function

/////////////// 1. peripheral clock setup (1 function) //////////////

/*********************************************************
  ----Function documentation--------------------------
  *@ Function  - I2C_PeriClockControl
  *@ Brief     - this function enable or disable peripheral clock for the given SPI port
  *@ param[in] - Base address of the SPI peripheral
  *@ param[in] - ENABLE or DISABLE macros
  *@ return    - void
  *@ note      - Using Else if cheaking
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

}

///////////// 2. init and De-init (3 function) /////////////////

uint32_t RCC_GetPLLOutputClock()
{
	return;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;

	}else if(clksrc == 1)
	{
		SystemClk = 8000000;

	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	return pclk1;

	// APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - I2C_Initialization
  *@ Brief     -
  *@ param[in] - Handle structure --> pSPIx, SPI_Config --> structure
  *@ return    - void
  *@ note      - none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg  = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// program the device Own Address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR Calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standerd mode'
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

}
