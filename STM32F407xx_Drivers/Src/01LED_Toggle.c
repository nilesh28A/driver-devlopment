/*
 * 01LED_Toggle.c
 *
 *  Created on: 21-Jul-2023
 *      Author: niles
 */

#include"STM32F4xx.h"

void delay(void){
	for (uint32_t i=0;i<50000;i++);
}

int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
