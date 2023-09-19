/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: June 30, 2023
 *      Author: nilesh A
 */

#include "stm32f407xx_gpio_driver.h"

// APIs implemenration through function

/////////////// 1. peripheral clock setup (1 function) //////////////

/*********************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_PeriClockControl
  *@ Brief     - this function enable or disable peripheral clock for the given GPIO port
  *@ param[in] - Base address of the GPIO peripheral
  *@ param[in] - ENABLE or DISABLE macros
  *@ return    - void
  *@ note      - Using Else if cheaking
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}

}


///////////// 2. init and De-init (2 function) /////////////////

/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_Initialization
  *@ Brief     - MOde, speed ,pu/pd, output type, alt function
  *@ param[in] - Handle structure --> pGPIOx, GPIO_PinConfig --> structure
  *@ return    - void
  *@ note      - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//***** 2.1 configure the mode og GPIO pin
	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
		temp = 0;
	}else
	{
		// this part will code latter (intrrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear tne coresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear tne coresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// set tne coresponding RTSR bit
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIOP port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the exti intrrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//***** 2.2 configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting
	temp = 0;

	//***** 2.3 configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;  //setting
	temp = 0;

	//***** 2.4 configure the optypes
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;  //setting
	temp = 0;

	//***** 2.5 configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the alternative function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 =	pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}


/*****************************************************
 ----Function documentation--------------------------
  *@ Function  - GPIO_DeInit
  *@ Brief     - reset all the register of peripherals
  *@ param[in] - Gpio port name
  *@ return    - void
  *@ note      - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_PCLK_EN();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}


//////////////// 3. data read write (5 function) //////////////////
/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ReadFromInputPin
  *@ Brief     -
  *@ param[in] - Gpio port name
  *@ param[in] - Gpio pin number
  *@ return    - value of pin number 0/1
  *@ note      - none
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ReadFronInputPort
  *@ Brief     -
  *@ param[in] - port name
  *@ return    - value
  *@ note      - none
 */
uint16_t GPIO_ReadFronInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_WriteToOutputPin
  *@ Brief     -
  *@ param[in] - gpio port name
  *@ param[in] - gpio pin number
  *@ param[in] - value 0/1
  *@ return    - void
  *@ note      - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET){
		//Write 1 to the output dta register at the bit feild corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_WriteToOutputPort
  *@ Brief     -
  *@ param[in] - Port name
  *@ param[in] - value
  *@ return    - void
  *@ note      - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR == value;
}


/*****************************************************
  ----Function documentation--------------------------
  *@ Function  - GPIO_ToggleOutputPin
  *@ Brief     -
  *@ param[in] - port name
  *@ param[in] - pin number
  *@ return    - void
  *@ note      - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/////////// 4. IRQ Configuration and IRQ Handling (2 functions) ///////////////////////////
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
  *@ Brief     -
  *@ param[in] -
  *@ param[in] -
  *@ param[in] -
  *@ return    - none
  *@ note      - none
 */
void GPIO_IRQHandling(uint8_t PinNumber){

}



