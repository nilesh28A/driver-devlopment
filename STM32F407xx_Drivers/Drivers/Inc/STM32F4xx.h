/*
 * STM32F4xx.h
 *
 *  Created on: July 17, 2023
 *      Author: niles
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include<stdint.h>
#define __vo volatile

//1. base address of flash and SRAM
#define FLASH_BASEADDR    0x08000000U
#define SRAM1_BASEADDR    0x20000000U  //112KB
#define SRAM2_BASEADDR    0x20001C00U
#define ROM               0x1FFF0000U
#define SRAM              SRAM1_BASEADDR


//2. AHBx and APBx
#define PERIPH_BASEADDR       0x40000000U
#define APB1PERIPH_BASEADDR   PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR   0x40010000U
#define AHB1PERIPH_BASEADDR   0x40020000U
#define AHB2PERIPH_BASEADDR   0x50000000U


//3. base address of hanging on AHB1 bus
#define GPIOA_BASEADDR    (APB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR    (APB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR    (APB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR    (APB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR    (APB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR    (APB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR    (APB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR    (APB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR    (APB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR      (APB1PERIPH_BASEADDR + 0x3800)

//4. base address of hanging on APB1 bus
#define I2C1_BASEADDR     (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR     (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR     (APB1PERIPH_BASEADDR + 0x5C00)


#define SPI2_BASEADDR     (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR     (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR     (APB1PERIPH_BASEADDR + 0x3400)

#define USART2_BASEADDR   (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR   (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR    (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR    (APB1PERIPH_BASEADDR + 0x5000)


//5. base address of hanging on APB2 bus
#define EXIT_BASEADDR     (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR     (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR   (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR   (APB2PERIPH_BASEADDR + 0x3000)
#define USART6_BASEADDR   (APB2PERIPH_BASEADDR + 0x1400)


//******************************************** 6. Peripheral Register definition structure ************************************

// 6.1 ************* structure for GPIO
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

// 6.1 ************* structure for EXTI
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXIT_RegDef_t;


//************* structure for SYSCONFIG
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;



// ******************* structure for SYSCONFIG


// 6.1.1  peripheral definitions (type casted)
#define GPIOA            ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB            ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC            ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD            ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE            ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF            ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG            ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH            ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI            ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXIT             ((EXTI_RegDef_t*)EXIT_BASEADDR)
#define SYSCFG           ((EXTI_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1             ((SPI1_RegDef_t*)SPI1_BASEADDR)
#define SPI2             ((SPI1_RegDef_t*)SPI2_BASEADDR)
#define SPI3             ((SPI1_RegDef_t*)SPI3_BASEADDR)

// 6.2 ************* structure for RCC
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	     uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	     uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	     uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	     uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	     uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	     uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	     uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_RegDef_t;

// 6.2.1  peripheral definitions (type casted)
#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)

// 7. clock enable macros

// 7.1 clock enable macros for GPIOx peripheral
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |=(1<<8))

// 7.2 clock enable macros for I2Cx peripherals
#define I2C1_PCLK_EN()   (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |=(1<<23))

// 7.3 clock enable macros for SPIx peripherals
#define SPI1_PCLK_EN()   (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()   (RCC->APB2ENR |=(1<<14))
#define SPI3_PCLK_EN()   (RCC->APB2ENR |=(1<<15))
#define SPI4_PCLK_EN()   (RCC->APB2ENR |=(1<<13))

// 7.4 clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()   (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()   (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()   (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN()   (RCC->APB2ENR |=(1<<5))

// 7.5 clock enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |=(1<<14))

// 8. clock disable macros

// 8.1 clock disable macros for GPIOx peripheral
#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()  (RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()  (RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()  (RCC->AHB1ENR &=~(1<<8))

// 8.2 clock disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()   (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &=~(1<<23))

// 8.3 clock disable macros for SPIx peripherals
#define SPI1_PCLK_DI()   (RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()   (RCC->APB2ENR &=~(1<<14))
#define SPI3_PCLK_DI()   (RCC->APB2ENR &=~(1<<15))
#define SPI4_PCLK_DI()   (RCC->APB2ENR &=~(1<<13))

// 8.4 clock disable macros for USARTx peripherals
#define USART1_PCLK_DI()   (RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()   (RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()   (RCC->APB1ENR &=~(1<<18))
#define USART4_PCLK_DI()   (RCC->APB1ENR &=~(1<<19))
#define USART5_PCLK_DI()   (RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()   (RCC->APB2ENR &=~(1<<5))

// 8.5 clock disable macros for SYSCFG peripherals
#define SYSCFG_PCLK_DI()   (RCC->APB2ENR &=~(1<<14))

// Macros  to reset GPIOx peripherals
#define GPIOA_REG_RESET()  do{(RCC->APB2ENR |= (1<<0)); (RCC->APB2ENR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()  do{(RCC->APB2ENR |= (1<<1)); (RCC->APB2ENR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()  do{(RCC->APB2ENR |= (1<<2)); (RCC->APB2ENR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()  do{(RCC->APB2ENR |= (1<<3)); (RCC->APB2ENR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()  do{(RCC->APB2ENR |= (1<<4)); (RCC->APB2ENR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET()  do{(RCC->APB2ENR |= (1<<5)); (RCC->APB2ENR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET()  do{(RCC->APB2ENR |= (1<<6)); (RCC->APB2ENR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET()  do{(RCC->APB2ENR |= (1<<7)); (RCC->APB2ENR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET()  do{(RCC->APB2ENR |= (1<<8)); (RCC->APB2ENR &= ~(1<<8));} while(0)


// this macros to return a code for a given GPIO base address
#define GPIO_BASEADDR_TO_CODE(x)      ((x==GPIOA)?0:\
		                               (x==GPIOB)?1:\
                                       (x==GPIOC)?2:\
		                               (x==GPIOD)?3:\
			                           (x==GPIOE)?4:\
						               (x==GPIOF)?5:\
						        	   (x==GPIOG)?6:\
									   (x==GPIOH)?7:\
								       (x==GPIOI)?8:0)

// some generic macros
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESER           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#include "STM32F4xx_GPIO_Drivers.h"

#endif /* INC_STM32F4XX_H_ */
