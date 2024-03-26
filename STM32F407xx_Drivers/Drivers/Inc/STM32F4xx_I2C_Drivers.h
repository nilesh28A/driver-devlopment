/*
 * STM32F407xx_I2C_Drivers.h
 *
 *  Created on: 28-Oct-2023
 *      Author: niles
 */

#ifndef INC_STM32F407XX_I2C_DRIVERS_H_
#define INC_STM32F407XX_I2C_DRIVERS_H_

#include "STM32F407xx.h"

// ********1. configuration structure of I2C
typedef struct{
	uint8_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;


// ********2. handle structure of I2C
typedef struct{
	I2C_RegDef_t   *pI2Cx;
	I2C_Config_t    I2C_Config;
}I2C_Handle_t;


//********* 3. macros related to I2C Registers

// @I2C_SCLSpeed
#define I2C_SCL_SPEED_SM     100000
#define I2C_SCL_SPEED_FM2K   200000
#define I2C_SCL_SPEED_FM4K   400000

// @I2C_ACKControl
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0

// @I2C_FMDutyCycle
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1



//*******************************************************************************************************************
//**                         4. functions for APIs supported by the drivers                                        **
//**               for mor information about the APIs check the function defination                                **
//*******************************************************************************************************************

// 4.1 peripheral clock setup
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// 4.2 Init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// 4.3 Data Send and Recive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);




// 4.4 I2CIRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

// 4.5 other peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

// 4.6 application call back
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_DRIVERS_H_ */
