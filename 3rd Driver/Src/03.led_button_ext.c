/*
 * 03.led_button_ext.c
 *
 *  Created on: 31-June-2023
 *      Author: niles
 */

#include"stm32f407xx.h"
#include"stm32f407xx_gpio_driver.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for (uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;

	// for led
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // TRY BOTH OD/PP
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

	// for button
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}
	}

}

