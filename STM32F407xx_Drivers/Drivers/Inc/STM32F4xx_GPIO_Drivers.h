/*
 * STM32F4xx_GPIO_Drivers.h
 *
 *  Created on: 18-Jul-2023
 *      Author: niles
 */

#ifndef INC_STM32F4XX_GPIO_DRIVERS_H_
#define INC_STM32F4XX_GPIO_DRIVERS_H_

#include "STM32F4xx.h"

// 9. configuration structure of GPIO pin
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


// 10. handle structure of GPIO
typedef struct{

	//pointer to hold the base address of the GPIO peripherals
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


// GPIO pin numbers
#define GPIO_PIN_NO_0  0
#define GPIO_PIN_NO_1  1
#define GPIO_PIN_NO_2  2
#define GPIO_PIN_NO_3  3
#define GPIO_PIN_NO_4  4
#define GPIO_PIN_NO_5  5
#define GPIO_PIN_NO_6  6
#define GPIO_PIN_NO_7  7
#define GPIO_PIN_NO_8  8
#define GPIO_PIN_NO_9  9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


// GPIO pin Possible mode
#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4   // falling
#define GPIO_MODE_IT_RT  5   // rising
#define GPIO_MODE_IT_RFT 6   // rising & falling


// GPIO pin possible output type
#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1


// GPIO pin possible output speed
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MIDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3


// GPIO pull up & pull down macros
#define GPIO_NO_PUPD   0
#define GPIO_PIN_PU    1
#define GPIO_PIN_PD    2



//******************11. functions for APIs supported by the drivers ******************************************

// 11.1 peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// 11.2 Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// 11.3 data read write
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFronInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// 11.4 IRQ Configuration and ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F4XX_GPIO_DRIVERS_H_ */
