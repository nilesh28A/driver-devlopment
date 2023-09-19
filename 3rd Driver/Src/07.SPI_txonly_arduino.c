/*
 * 07.SPI_txonly_arduino.c
 *
 *  Created on: 19-july-2023
 *      Author: niles
 */

#include<stdio.h>
#include<stdint.h>
#include<string.h>

#include"stm32f407xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for (uint32_t i=0;i<500000;i++);
}

/*
 * PB14--> SPI2_MISO
 * PB15--> SPI2_MOSI
 * PB13--> SPI2_SCLK
 * PB12--> SPI2_NSS
 * ATL Function Mode := 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;       // 2 MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;      // hard slave mange enable foe NSS

	SPI_Init(&SPI2Handle);
}

GPIO_ButtonInit()
{
	GPIO_Handle_t  GPIOBtn;

	// for button gpio config
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
}

int main ()
{
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	// this fun is used to initialize the GPIO pins to behave as SPI2 Pins
	SPI2_GPIOInits();

	// this fun is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	//this make NSS singnal internal high and avoid MODF error
	//SPI_SSIConfig(SPI2, ENABLE); only for soft slave mange

	while(1)
	{
		// wait till button is press
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// TO avoid de-bouncing
		delay();

		//enable the SPI2 peripherals
		SPI_PeripheralControl(SPI2, ENABLE);

		// to send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//disable the SPI2 peripherals
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

