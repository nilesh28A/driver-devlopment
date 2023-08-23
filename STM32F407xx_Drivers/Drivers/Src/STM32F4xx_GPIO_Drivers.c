/*
 * STM32F4xx_GPIO_Drivers.C
 *
 *  Created on: 18-Jul-2023
 *      Author: niles
 */


#include "STM32F4xx_GPIO_Drivers.h"

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

//******************************** 11.  APIs supported by the drivers ****************************************

// 11.1  peripheral clock setup
/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_PeriClockControl
  *
  *@ Brief     - this function enable or disable peripheral clock for the given GPIO port
  *
  *@ param[in] - Base address of the GPIO peripheral
  *@ param[in] - ENABLE or DISABLE macros
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(GPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(GPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(GPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(GPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(GPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(GPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(GPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(GPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(GPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else{
		if(GPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(GPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(GPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(GPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(GPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(GPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(GPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(GPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(GPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

// 11.2 init and De-init

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_Init
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t tmep = 0;
	//1.configure the mode of GPIO pin
	if(pGPIOHandl->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		//the none interrupt mode
		tmep = (pGPIOHandl->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandl->GPIOx->MODER &= ~(0x3 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandl->GPIOx->MODER |= temp;

	}else{
		//interrupt mode configuration
		if(pGPIOHandl->GPIO_PinConfig.GPIO_PinMode == GPIO_OMDE_IT_FT){
			//1. Configure the FTSR
			EXIT->FTSR |= (1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);
			EXIT->RTSR &= ~(1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandl->GPIO_PinConfig.GPIO_PinMode == GPIO_OMDE_IT_RT){
			//1. Configure the RTSR
			EXIT->RTSR |= (1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);
			EXIT->FTSR &= ~(1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandl->GPIO_PinConfig.GPIO_PinMode == GPIO_OMDE_IT_RFT){
			//1. Configure the both FTSR & RTSR
			EXIT->RTSR |= (1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);
			EXIT->FTSR |= (1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);

			//2. configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandl->GPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (temp2*4);


			//3. Enable the exit interrupt delivery using IMR
			EXIT->IMR |= (1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);
		}
	}
	tmep = 0;

	//2.configure the speed
	tmep = (pGPIOHandl->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandl->GPIOx->OSPEEDR &= ~(0x3 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandl->GPIOx->OSPEEDR |= temp;
	tmep = 0;

	//3.configure the pupd settings
	tmep = (pGPIOHandl->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandl->GPIOx->PUPDR &= ~(0x3 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandl->GPIOx->PUPDR |= temp;
	tmep = 0;

	//4.configure the optypes
	tmep = (pGPIOHandl->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandl->GPIOx->OTYPER &= ~(0x1 << pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandl->GPIOx->OTYPER |= temp;
	tmep = 0;

	//5.configure the alt functionality
	if(pGPIOHandl->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//configure the alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 =	pGPIOHandl->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandl->GPIOx->AFR[temp1] &= ~(pGPIOHandl->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
		pGPIOHandl->GPIOx->AFR[temp1] |= (pGPIOHandl->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_DeInit
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(GPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(GPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(GPIOx == GPIOC){
		GPIOC_PCLK_EN();
	}else if(GPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(GPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(GPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(GPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(GPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(GPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


// 11.3 data read write

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ReadFromInputPin
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ReadFronInputPort
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
uint16_t GPIO_ReadFronInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - WriteToOutputPin
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(value == GPIO_PIN_SET){
		//Write 1
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		//Write 0
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_WriteToOutputPort
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOPx->ODR == value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ToggleOutputPin
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}


// 11.4 IRQ Configuration and ISR Handling

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_IRQConfig
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_IRQHandling
  *
  *@ Brief     -
  *
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *
  *@ return    - none
  *
  *@ note      - none
 */
void GPIO_IRQHandling(uint8_t PinNumber){

}
