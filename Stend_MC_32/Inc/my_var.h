#ifndef __my_var_H
#define __my_var_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"	 
/* USER CODE END Includes */


//УПРАВЛЕНИЕ	КЛЮЧАМИ 
//управление ключом 1
#define Key1_On GPIOB,GPIO_PIN_13,GPIO_PIN_SET
#define Key1_Off GPIOB,GPIO_PIN_13,GPIO_PIN_RESET
//управление ключом 2
#define Key2_On GPIOB,GPIO_PIN_12,GPIO_PIN_SET
#define Key2_Off GPIOB,GPIO_PIN_12,GPIO_PIN_RESET
//управление ключом 3
#define Key3_On GPIOB,GPIO_PIN_3,GPIO_PIN_SET
#define Key3_Off GPIOB,GPIO_PIN_3,GPIO_PIN_RESET 
//управление ключом 4
#define Key4_On GPIOB,GPIO_PIN_14,GPIO_PIN_SET
#define Key4_Off GPIOB,GPIO_PIN_14,GPIO_PIN_RESET 
//управление ключом 5
#define Key5_On GPIOB,GPIO_PIN_5,GPIO_PIN_SET
#define Key5_Off GPIOB,GPIO_PIN_5,GPIO_PIN_RESET 
//управление ключом 6
#define Key6_On GPIOB,GPIO_PIN_4,GPIO_PIN_SET
#define Key6_Off GPIOB,GPIO_PIN_4,GPIO_PIN_RESET 
//управление ключом 7
#define Key7_On GPIOB,GPIO_PIN_7,GPIO_PIN_SET
#define Key7_Off GPIOB,GPIO_PIN_7,GPIO_PIN_RESET 
//управление ключом 8
#define Key8_On GPIOB,GPIO_PIN_6,GPIO_PIN_SET
#define Key8_Off GPIOB,GPIO_PIN_6,GPIO_PIN_RESET
//управление ключом 9
#define Key9_On GPIOB,GPIO_PIN_9,GPIO_PIN_SET
#define Key9_Off GPIOB,GPIO_PIN_9,GPIO_PIN_RESET 
//управление ключом 10
#define Key10_On GPIOB,GPIO_PIN_8,GPIO_PIN_SET
#define Key10_Off GPIOB,GPIO_PIN_8,GPIO_PIN_RESET 
//управление ключом 11
#define Key11_On GPIOB,GPIO_PIN_10,GPIO_PIN_SET
#define Key11_Off GPIOB,GPIO_PIN_10,GPIO_PIN_RESET 
//управление ключом 12
#define Key12_On GPIOB,GPIO_PIN_11,GPIO_PIN_SET
#define Key12_Off GPIOB,GPIO_PIN_11,GPIO_PIN_RESET 
//светодиод LED3
#define LED3_On GPIOA,GPIO_PIN_15,GPIO_PIN_SET
#define LED3_Off GPIOA,GPIO_PIN_15,GPIO_PIN_RESET 
#define LED3_Toggle GPIOA,GPIO_PIN_15
//Регулятор давления бит 1
#define RM_bit1_Set GPIOA,GPIO_PIN_4,GPIO_PIN_SET
#define RM_bit1_Reset GPIOA,GPIO_PIN_4,GPIO_PIN_RESET
//Регулятор давления бит 2
#define RM_bit2_Set GPIOA,GPIO_PIN_5,GPIO_PIN_SET
#define RM_bit2_Reset GPIOA,GPIO_PIN_5,GPIO_PIN_RESET
//Регулятор давления бит 3
#define RM_bit3_Set GPIOA,GPIO_PIN_6,GPIO_PIN_SET
#define RM_bit3_Reset GPIOA,GPIO_PIN_6,GPIO_PIN_RESET
//Регулятор давления бит 4
#define RM_bit4_Set GPIOA,GPIO_PIN_7,GPIO_PIN_SET
#define RM_bit4_Reset GPIOA,GPIO_PIN_7,GPIO_PIN_RESET
//Регулятор давления бит 5
#define RM_bit5_Set GPIOB,GPIO_PIN_0,GPIO_PIN_SET
#define RM_bit5_Reset GPIOB,GPIO_PIN_0,GPIO_PIN_RESET
//Регулятор давления бит 6
#define RM_bit6_Set GPIOB,GPIO_PIN_1,GPIO_PIN_SET
#define RM_bit6_Reset GPIOB,GPIO_PIN_1,GPIO_PIN_RESET
//Регулятор давления бит 7
#define RM_bit7_Set GPIOA,GPIO_PIN_8,GPIO_PIN_SET
#define RM_bit7_Reset GPIOA,GPIO_PIN_8,GPIO_PIN_RESET
//Регулятор давления бит 8
#define RM_bit8_Set GPIOB,GPIO_PIN_15,GPIO_PIN_SET
#define RM_bit8_Reset GPIOB,GPIO_PIN_15,GPIO_PIN_RESET


extern uint8_t rx1_byte[9];
extern uint8_t rx2_byte[4];

extern xQueueHandle UART1_Rx_Que;
extern portBASE_TYPE xStatus_Rx_UART1;
extern xQueueHandle UART2_Rx_Que;
extern portBASE_TYPE xStatus_Rx_UART2;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__my_var_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
