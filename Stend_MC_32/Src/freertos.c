/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "adc.h"
#include "rtc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "my_var.h"
#include "math.h"

//проверка что в бит установленна единица
#define bitRead(data,bitno)            ((data >> bitno) & 0x01)
#define bitSet(data, bit)              ((data) |= (1UL << (bit)))
#define bitClear(data, bit)            ((data) &= ~(1UL << (bit)))
#define bitWrite(data, bit, bitvalue)  (bitvalue ? bitSet(data, bit) : bitClear(data, bit))

// преобразование из uint16_t в uint8_t
#define BYTE_0(n) ((uint8_t)((n) & (uint8_t)0xFF))        /*!< Returns the low byte of the 16-bit value */
#define BYTE_1(n) ((uint8_t)(BYTE_0((n) >> (uint8_t)8)))  /*!< Returns the second byte of the 16-bit value */
/* USER CODE END Includes */

// ключи на клапана и двигатели
#define Klapan_1_on Key2_On
#define Klapan_1_off Key2_Off
#define Klapan_2_on Key1_On
#define Klapan_2_off Key1_Off
#define Klapan_3_on Key4_On
#define Klapan_3_off Key4_Off
#define Klapan_4_on Key3_On
#define Klapan_4_off Key3_Off
#define Klapan_5_on Key6_On
#define Klapan_5_off Key6_Off
#define Klapan_6_on Key5_On
#define Klapan_6_off Key5_Off
#define Engine_1_on Key10_On
#define Engine_1_off Key10_Off
#define Engine_2_start_on Key9_On
#define Engine_2_start_off Key9_Off
#define Engine_2_reverse_on Key11_On
#define Engine_2_reverse_off Key11_Off
#define Engine_2_stop_on Key12_On
#define Engine_2_stop_off Key12_Off
//чтение клапанов и двигателя
#define Klapan_1 GPIOB,GPIO_PIN_12
#define Klapan_2 GPIOB,GPIO_PIN_13
#define Klapan_3 GPIOB,GPIO_PIN_14
#define Klapan_4 GPIOB,GPIO_PIN_3
#define Klapan_5 GPIOB,GPIO_PIN_4
#define Klapan_6 GPIOB,GPIO_PIN_5
#define Engine_1 GPIOB,GPIO_PIN_8
#define Engine_2_start GPIOB,GPIO_PIN_9
#define Engine_2_stop GPIOB,GPIO_PIN_10
#define Engine_2_reverse GPIOB,GPIO_PIN_11
// ДОБАВИТЬ РЕВЕРС

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId myTask08Handle;
osThreadId myTask09Handle;
osThreadId myTask10Handle;
osThreadId myTask11Handle;

/* USER CODE BEGIN Variables */

uint8_t test_i = 0;
//union {
//   float a;
//   unsigned char bytes[4];
//} thing;
//{0xD6,0x6D,0x42,0x9E}; DC BA
unsigned char bytes[4] = {0x6D,0xD6,0x9E,0x42};
//volatile float float_variable = 0;
//volatile uint16_t crc_modbus = 0;

float liquid_level_sensor = 0;  //значение снятое с АЦП 1 канала
//усредняем значение
float sum_liquid_level_sensor = 0;
//истинное значение
float true_liquid_level_sensor = 0;

float temp_sensor = 0;  //значение снятое с АЦП 2 канала
//усредняем значение
float sum_temp_sensor = 0;
//истинное значение
float true_temp_sensor = 0;


uint8_t temperature = 0; //значение температуры
uint8_t liquid_level = 0; //уровень жидкости
uint16_t moment = 0; //момент
float moment_float = 0; //момент
uint16_t pressure = 0; //давление

uint8_t ReceiveData[2];
char str_rx[2];

xQueueHandle UART1_Tx_Que;//очередь на отправку в UART1
portBASE_TYPE xStatus_Tx_UART1;
uint8_t tx1_buffer_que[10];//буфер для очереди на отправку в UART1

xQueueHandle UART2_Tx_Que;//очередь на отправку в UART2
portBASE_TYPE xStatus_Tx_UART2;
uint8_t tx2_buffer_que[10];//буфер для очереди на отправку в UART2

HAL_StatusTypeDef TransmitStatusUART1 = HAL_TIMEOUT;
uint16_t len_tx1_buffer = 1;
uint8_t tx1_buffer_go[10];

HAL_StatusTypeDef TransmitStatusUART2 = HAL_TIMEOUT;
uint16_t len_tx2_buffer = 0;
uint8_t tx2_buffer_go[4];

uint8_t test1 = 0;
// Статус включения двигателя
uint8_t Engine_1_State = 0;
uint8_t Engine_2_StateStart = 0;
uint8_t Engine_2_StateStop = 0;
uint8_t Engine_2_StateReverse = 0;
uint8_t Klapan_6_State = 0;

HAL_StatusTypeDef TransmitStatusUSB = HAL_TIMEOUT;
uint8_t DataToSendUSB_1[8];
uint8_t DataToSendUSB_2[8];
uint8_t DataToSendUSB_all[24];
// массив с состоянием ключей
uint8_t key_read[6];
uint8_t i = 0;
// Флаг проверки коректного принятого сообщения
uint8_t new_message_flag = 0;
uint8_t rx_usb_message_count = 0; //кол-во элементов в буфере UART_1
uint8_t rx_usb_message[64]; //буфер UART_1
//флаг статуса работы
uint8_t hello = 0;
uint8_t book_load = 0;
uint8_t param_load = 0;
uint8_t exper_run = 0; // 1 - испытание 1, 2 - испытание 2, 3-испытание закончено, 0-испытание нет
uint8_t i_exper = 0;
uint8_t error_status = 0;
uint8_t error_status_buf = 0;
// Команды на включение двигателей
// 0=выкл,1=вкл,
uint8_t Engine_1_Command = 0;
// 0=выкл,1=вкл,-1=реверс 
int8_t Engine_2_Command = 0;
//запрос ответа
uint8_t request_answer = 0;
//запрос данных
uint8_t request_data = 0;
// счетчик бит для ответов
uint8_t hearth_bit_answer = 0;
uint8_t old_hearth_bit_answer = 0;
//остановка по команде оператора
uint8_t flag_operator_stop = 0;
//остановка флаг аварийного режима
uint8_t flag_emergency_mode = 0;
// время до окончания аварийного режима
uint16_t time_emergency_mode = 0;
// параметры сценария = {3, 4, 5, 6, 7, 8, 9, 10, 11, 10};
uint16_t book_time[200];
uint16_t book_pressure[200] = {
 0,   0,  0,  5, 10, 15, 20, 25, 30, 35,
 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 
 90, 95,100,105,110,115,120,125,130,135,
140,145,150,155,160,165,170,175,180,185,
190,195,200,195,190,185,180,175,170,165,
160,155,150,145,140,135,130,125,120,115, 
110,105,100, 95, 90, 85, 80, 75, 70, 65, 
 60, 55, 50, 45, 40, 35, 30, 25, 20, 15,  
 10,  5,  0,  0,  0,  0,  5, 10, 15, 20, 25, 30, 35,
 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 
 90, 95,100,105,110,115,120,125,130,135,
140,145,150,155,160,165,170,175,180,185,
190,195,200,195,190,185,180,175,170,165,
160,155,150,145,140,135,130,125,120,115, 
110,105,100, 95, 90, 85, 80, 75, 70, 65, 
 60, 55, 50, 45, 40, 35, 30, 25, 20, 15,  
 10,  5,  0,  0,  0,  0};
uint8_t book_valve_state[200] = {
18, 37, 37, 37, 37, 37, 37, 37, 37, 37,
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 
37, 37, 37, 37, 37, 37, 37, 37, 37, 37,   
37, 37, 37, 32, 42, 42, 42, 42, 42, 42,  
42, 42, 42, 42, 42, 42, 42, 42, 42, 42,  
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 42, 42, 42, 42,  
42, 42, 42, 42, 42, 42, 42, 42, 42, 42,  
42, 42, 42, 42, 42, 42, 42, 42, 42, 42,  
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42,  0, 20,  0};
uint8_t book_engine_state[200]= {
 2,  0,  1,  1,  1,  1,  1,  1,  1,  1,
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,   
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1, 
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
 1,  1,  1,  1,  1,  1,  0,  4,  0};

// параметры ЭД2
uint8_t t_zap_akv = 60;
uint8_t t_otk_akv = 60;
uint8_t h_min = 30;
uint8_t h_max_1 = 81;
uint8_t h_max_2 = 70;
uint8_t h_max = 81;
 
uint16_t book_len_p1 = 0;
uint16_t book_len_p2 = 0;
uint16_t book_len = 169;
uint8_t book_i = 0;
//контрольная сумма
uint16_t my_crc = 0;
// время эксперимента
uint16_t exper_time = 0;
// счетчик ошибок связи
uint8_t miss_communication_count = 0;
// давление для отправки
float p_value_f = 0;
uint8_t p_value_bit = 0;
uint16_t p_value_rs232 = 0;

uint8_t rx1_buffer_count = 0; //кол-во элементов в буфере UART_1
uint8_t rx1_buffer[64]; //буфер UART_1

uint8_t rx1_message[9];
uint16_t rx1_message_count = 0;

// Флаг проверки коректного принятого сообщения
uint8_t new_message_flag_uart = 0;
// задаваемое давление
float sended_pressure = 0;
// проверки связи с ZETLAB
uint8_t zetlab_not_connected = 0;
uint16_t zetlab_wait_answer = 0;
uint8_t moment_request = 0;
// проверки связи с РД
uint8_t pressure_not_connected = 0;
uint16_t pressure_wait_answer = 0;
uint8_t pressure_request = 0;

uint16_t Hearth_Bit = 0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void Read_ADC(void const * argument);
void Read_USB(void const * argument);
void Send_UART1(void const * argument);
void Send_UART2(void const * argument);
void Send_USB(void const * argument);
void Exper_run(void const * argument);
//void Read_UART(void const * argument);
void ReceiveUART1(void const * argument);
void ReceiveUART2(void const * argument);
void CheckError(void const * argument);
void FillLiquid(void const * argument);
//void MeanADC(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void Test(char w);
void ClearBuffer(uint8_t *buffer,uint16_t *len_buffer);
void Send_Pressure_RS232(uint16_t value);
void Control_Engine_2(int8_t command,uint8_t Klapan_6_Readed);
void My_Delay(uint32_t time_delay);
void My_Emergency(uint32_t time_delay);

uint16_t CRC16_2(unsigned char *buf, int len);
uint16_t crc_16_ibm(uint8_t *buf, uint16_t len);
uint8_t Check_crc_16_modbus(uint8_t *buf, uint16_t len);
uint8_t Check_crc_16(uint8_t *buf, uint16_t len);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	book_time[0] = 0;
	book_time[1] = 400;
	uint16_t now_time = 400;
	//Заполнение массива времени
  for (uint16_t i_book_time=2;i_book_time<book_len-1;i_book_time++){
	  book_time[i_book_time] = now_time+((i_book_time-1)*30);
	}
	book_time[168] = 5900;
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(myTask02, Read_ADC, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);
	
  osThreadDef(myTask03, Read_USB, osPriorityLow, 0, 256);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);
	
  osThreadDef(myTask04, Send_UART1, osPriorityBelowNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);
	
	osThreadDef(myTask05, Send_USB, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);
	
	osThreadDef(myTask06, Exper_run, osPriorityLow, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);
	
	osThreadDef(myTask07, Send_UART2, osPriorityAboveNormal, 0, 128);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);
	
	osThreadDef(myTask08, ReceiveUART1, osPriorityHigh, 0, 128);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);
	
	osThreadDef(myTask09, ReceiveUART2, osPriorityHigh, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);
	
	osThreadDef(myTask10, CheckError, osPriorityNormal, 0, 128);
  myTask10Handle = osThreadCreate(osThread(myTask10), NULL);
	
	osThreadDef(myTask11, FillLiquid, osPriorityLow, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	UART1_Tx_Que = xQueueCreate(5, sizeof(tx1_buffer_que));
	UART2_Tx_Que = xQueueCreate(5, sizeof(tx2_buffer_que));
  /* add queues, ... */
	
	//osDelay(10000);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
	HAL_GPIO_WritePin(LED3_On);
	
//	float_variable = *(float *)&bytes;
//	crc_modbus = CRC16_2(&bytes[4], 4);
	// подадим давление 0 Бар на регулятор
	Send_Pressure_RS232(0);
	
	uint16_t ADC_2V = 2423; // это 100 %
	uint16_t ADC_0_4V = 432; // это 0%
	float TEMP_k = 0.197;
	float TEMP_b = -89.375;
	uint8_t i_adc_l = 0; 
	uint8_t i_adc_t = 0; 
	
	uint8_t pressure_error_counter = 0;
  /* USER CODE BEGIN StartDefaultTask */
	
  /* Infinite loop */
  for(;;)
  {
		Hearth_Bit ++;
		// Температура
		if (i_adc_t >= 10){
		  true_temp_sensor = (sum_temp_sensor/100.0);
			// если на АЦП 0, то температура 0
		  if (true_temp_sensor != 0) {
				temperature = (uint8_t)roundf(((true_temp_sensor*TEMP_k) + TEMP_b));
				test_i = roundf(((true_temp_sensor*TEMP_k) + TEMP_b));
			}
		  else temperature = 0;
		  sum_temp_sensor = 0;
		  i_adc_t = 0;
		}
		else {
			i_adc_t++;
		}
		// Уровень жидкости
		if (i_adc_l >= 10){
		  true_liquid_level_sensor = sum_liquid_level_sensor/100.0;
		  sum_liquid_level_sensor = 0;
		  i_adc_l = 0;
			// если на АЦП 0, то уровень жидкости 0
		  if (true_liquid_level_sensor != 0) liquid_level = (uint8_t)(((true_liquid_level_sensor-ADC_0_4V)*100.0)/(ADC_2V-ADC_0_4V));
      else liquid_level = 0;
		  if (liquid_level>100) liquid_level = 100;
		}
		else {
			i_adc_l++;
		}

		tx1_buffer_que[0] = 0x05;
		tx1_buffer_que[1] = 0x03;
		tx1_buffer_que[2] = 0x00;
		tx1_buffer_que[3] = 0x14;
		tx1_buffer_que[4] = 0x00;
		tx1_buffer_que[5] = 0x02;
		tx1_buffer_que[6] = 0x85;
		tx1_buffer_que[7] = 0x8B;
		len_tx1_buffer = 8;
			/*
		  Для тестирования
			tx1_buffer_que[0] = 0x05;
		  tx1_buffer_que[1] = 0x05;
		  tx1_buffer_que[2] = 0x05;
		  tx1_buffer_que[3] = 0x05;
		  tx1_buffer_que[4] = 0x05;
		  tx1_buffer_que[5] = 0x05;
			tx1_buffer_que[6] = 0x05;
		  tx1_buffer_que[7] = 0x51;
		  tx1_buffer_que[8] = 0xCB;
		  len_tx1_buffer = 9;
			*/
		  //включаем прием сообщения
		  //HAL_UART_Receive_IT (&huart1,(uint8_t*)&rx1_byte,9);
		  // отправляем запрос момента
		xStatus_Tx_UART1 = xQueueSendToBack(UART1_Tx_Que,&tx1_buffer_que,0);
		//Проверяем пришел ли ответ на прошлый вопрос, если нет
    // устанавливаем флаг, что нет связи с ZETLAB
		if (zetlab_wait_answer >= 30){
			zetlab_not_connected = 1;
			zetlab_wait_answer = 30;
			moment = 0;
		}
		else zetlab_not_connected = 0;
		//Инкрементируем переменную ждем ответа
		// Обнуляем ее в приеме
		zetlab_wait_answer ++;
		//Давление
    osDelay(50);
		//запрос давления
	  tx2_buffer_que[0] = 0x02;
	  tx2_buffer_que[1] = 0x3F;
	  len_tx2_buffer = 2;
//		/*
//		Для тестирования
//		tx2_buffer_que[0] = 0x04;
//	  tx2_buffer_que[1] = 0xBF;
//		tx2_buffer_que[2] = 0x00;
//		tx2_buffer_que[3] ++;
//	  len_tx2_buffer = 4;
//		*/
	  //включаем прием сообщения
	  //HAL_UART_Receive_IT (&huart2,(uint8_t*)&rx2_byte,4);
	  // отправляем запрос давления
	  xStatus_Tx_UART2 = xQueueSendToBack(UART2_Tx_Que,&tx2_buffer_que,0);
		//Проверяем пришел ли ответ на прошлый вопрос, если нет
    // устанавливаем флаг, что нет связи с регулятором давления
		if (pressure_wait_answer >= 30){
			pressure_not_connected = 1;
			pressure = 0;
			pressure_wait_answer = 30;
			//HAL_UART_Receive_IT (&huart2,(uint8_t*)&rx2_byte,4);
		}
		else pressure_not_connected = 0;
		if (pressure_wait_answer >= 5) HAL_UART_Receive_IT (&huart2,rx2_byte,1);
		//Инкрементируем переменную ждем ответа
		// Обнуляем ее в приеме
		pressure_wait_answer ++;
		// отсчет времени эксперимента
		if (exper_run != 0)
		{
			HAL_GPIO_TogglePin(LED3_Toggle);
			// инкрементируем время
			exper_time ++;
			//проверка на ошибки
		  if (temperature>135)error_status = 1;
		  if (liquid_level<10)error_status = 2;
		  if (moment>700)error_status = 3;
		  if (pressure>400) pressure_error_counter ++;
			else pressure_error_counter = 0;
			if (pressure_error_counter >=10) {
				error_status = 4;
				pressure_error_counter = 10;
			}
			if (old_hearth_bit_answer == hearth_bit_answer){
				miss_communication_count++;
			}
			else miss_communication_count = 0;
		}
		else exper_time = 0;
		old_hearth_bit_answer = hearth_bit_answer;
		if (miss_communication_count>=20){
		  error_status = 5;
		  miss_communication_count = 0;
		}
    osDelay(50);
  }
  /* USER CODE END StartDefaultTask */
}

void CheckError(void const * argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN CheckError */
	
  /* Infinite loop */
  for(;;)
  {
		//Обработка ошибки
		if (error_status != 0)
		{
			exper_run = 0;
			miss_communication_count = 0;
			hello = 0;
			osDelay(100);
			// Эксперимент завершен по ошибке
			if (error_status == 1) DataToSendUSB_1[0] = 0xE2;
			if (error_status == 2) DataToSendUSB_1[0] = 0xE3;
			if (error_status == 3) DataToSendUSB_1[0] = 0xE4;
			if (error_status == 4) DataToSendUSB_1[0] = 0xE5;
			if (error_status == 5) DataToSendUSB_1[0] = 0xE6; // потеряна связь с компьютером
			DataToSendUSB_1[1] = 0x04;
			// Контрольная сумма
			my_crc = crc_16_ibm(DataToSendUSB_1,DataToSendUSB_1[1] - 2);
		  DataToSendUSB_1[2] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_1[3] = (uint8_t)(my_crc);
			// отправка сообщения
			CDC_Transmit_FS(DataToSendUSB_1, DataToSendUSB_1[1]);	
			osDelay(500);
			//выключение двигателей и закрытие клапанов
			HAL_GPIO_WritePin(Klapan_1_off);
			HAL_GPIO_WritePin(Klapan_2_off);
			HAL_GPIO_WritePin(Klapan_6_off);
			HAL_GPIO_WritePin(Engine_1_off);
			HAL_GPIO_WritePin(Engine_2_start_off);
			HAL_GPIO_WritePin(Engine_2_reverse_off);
			Klapan_6_State = 0;
			Send_Pressure_RS232(0);
			//включаем передачу данных после отправки команды о выключении и запоминаем ошибку
			error_status_buf = error_status;
			error_status = 0;
			hello = 1;
			flag_emergency_mode = 1;
			time_emergency_mode = 200*10;
			HAL_GPIO_WritePin(Klapan_3_on);
			HAL_GPIO_WritePin(Klapan_4_on);
			HAL_GPIO_WritePin(Klapan_5_on);
			My_Emergency(200*1000);
			HAL_GPIO_WritePin(Klapan_3_off);
			HAL_GPIO_WritePin(Klapan_4_off);
			HAL_GPIO_WritePin(Klapan_5_off);
			exper_run = 0;
			time_emergency_mode = 0;
			flag_emergency_mode = 0;
		}
    osDelay(100);
  }
  /* USER CODE END CheckError */
}
void FillLiquid(void const * argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN FillLiquid */
	
  /* Infinite loop */
  for(;;)
  {
		if (exper_run == 1)
		{
			if (((liquid_level<=h_min) || (Engine_2_Command == 1)) && Engine_2_Command != -1)
			{
       //заполнение жидкостью
       Engine_2_Command	= 1;			
			}
			if (liquid_level >= h_max && Engine_2_Command != -1) Engine_2_Command = 0;
			// вызов функции управления ЭД2 и клапан 6
			Control_Engine_2(Engine_2_Command,Klapan_6_State);
		}
		else if (Engine_2_Command == 0 && exper_run == 0 && flag_emergency_mode == 0){
	    //закрытие клапанов
	    HAL_GPIO_WritePin(Klapan_3_off);
	    HAL_GPIO_WritePin(Klapan_4_off);
	    HAL_GPIO_WritePin(Klapan_5_off);
		  Control_Engine_2(Engine_2_Command,Klapan_6_State);
			//Выключение ЭД2
		}
		if (Engine_2_Command == -1){
			 Control_Engine_2(Engine_2_Command,Klapan_6_State);
		}
    osDelay(100);
  }
  /* USER CODE END FillLiquid */
}
/* USER CODE BEGIN Application */
void Read_ADC(void const * argument) 
{
  /* USER CODE BEGIN Read_ADC */

  /* Infinite loop */
  for(;;)
  { 
		//считывание значений с АЦП
		HAL_ADCEx_InjectedStart_IT(&hadc1);
		temp_sensor =(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
		//усредняем значение температуры
		sum_temp_sensor = sum_temp_sensor + temp_sensor;
		
		liquid_level_sensor =(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2));
		//усредняем значение уровня жидкости
		sum_liquid_level_sensor = sum_liquid_level_sensor + liquid_level_sensor;		
		osDelay(10);
	}  /* USER CODE END Read_ADC */
}
void Exper_run(void const * argument) 
{
  /* USER CODE BEGIN Read_ADC */
  /* Infinite loop */
  for(;;)
  { 
		// все выключаем если не идет эксперимент
 		if (exper_run == 0 && Engine_2_Command == 0 && flag_emergency_mode == 0)
		{
			HAL_GPIO_WritePin(Klapan_1_off);
			HAL_GPIO_WritePin(Klapan_2_off);
			HAL_GPIO_WritePin(Klapan_3_off);
			HAL_GPIO_WritePin(Klapan_4_off);
			HAL_GPIO_WritePin(Klapan_5_off);
			HAL_GPIO_WritePin(Klapan_6_off);
			HAL_GPIO_WritePin(Engine_1_off);
			HAL_GPIO_WritePin(Engine_2_start_off);
			HAL_GPIO_WritePin(Engine_2_stop_off);
			HAL_GPIO_WritePin(Engine_2_reverse_off);
			Klapan_6_State = 0;
			//if (pressure > 0) Send_Pressure_RS232(0);
			Send_Pressure_RS232(0);
		}			
		//если пришел запрос на старт
		if (exper_run == 1)
		{ 
			for (i_exper=0;i_exper<(book_len);i_exper++)
		  {
				// ожидание
				// идет первым т.к. в массиве содержится время эксперимента, а не время ожидания
				if (i_exper == 0) My_Delay(book_time[i_exper]*100);
				else My_Delay((book_time[i_exper]-book_time[i_exper-1])*100);
				// изменения максимального уровня жидкости
				if (i_exper <= 1)	h_max = h_max_1;
				else h_max = h_max_2;
			
				// управление клапанами и двигателемя
				// состояние клапана 1
				if (bitRead(book_valve_state[i_exper],0)==1 && exper_run == 1) HAL_GPIO_WritePin(Klapan_1_on);
				else HAL_GPIO_WritePin(Klapan_1_off);
				// состояние клапана 2
				if (bitRead(book_valve_state[i_exper],1)==1 && exper_run == 1) HAL_GPIO_WritePin(Klapan_2_on);
				else HAL_GPIO_WritePin(Klapan_2_off);
				// состояние клапана 3
				if (bitRead(book_valve_state[i_exper],2)==1 && exper_run == 1) HAL_GPIO_WritePin(Klapan_3_on);
				else HAL_GPIO_WritePin(Klapan_3_off);
				// состояние клапана 4
				if (bitRead(book_valve_state[i_exper],3)==1 && exper_run == 1) HAL_GPIO_WritePin(Klapan_4_on);
				else HAL_GPIO_WritePin(Klapan_4_off);
				// состояние клапана 5
				if (bitRead(book_valve_state[i_exper],4)==1 && exper_run == 1) HAL_GPIO_WritePin(Klapan_5_on);
				else HAL_GPIO_WritePin(Klapan_5_off);
				// состояние клапана 6 работает по алгоритму слива и наполнения
				if (bitRead(book_valve_state[i_exper],5)==1 && exper_run == 1) Klapan_6_State = 1;
				else Klapan_6_State = 0;
				// состояние ЭД1
				if (bitRead(book_engine_state[i_exper],0)==1 && exper_run == 1) HAL_GPIO_WritePin(Engine_1_on);
				else HAL_GPIO_WritePin(Engine_1_off);
				// состояние ЭД2
				if (bitRead(book_engine_state[i_exper],1)==1 && exper_run == 1) Engine_2_Command = 1;
				else if (bitRead(book_engine_state[i_exper],2)==1 && exper_run == 1) Engine_2_Command = -1;
				else Engine_2_Command = 0;
				//Отправка давления для подпоршневой зоны в регулятор давления
				sended_pressure = book_pressure[i_exper];
				Send_Pressure_RS232(book_pressure[i_exper]);
				// выход по ошибке
				if (exper_run == 0) i_exper = book_len;
			}
			//Окончание испытания
			if (exper_run == 1) exper_run = 3;
		}
		HAL_GPIO_TogglePin(LED3_Toggle);
		osDelay(500);
	}  /* USER CODE END Read_ADC */
}
void Send_USB(void const * argument) 
{
  /* USER CODE BEGIN Read_ADC */
  const uint16_t PERIOD_TO_SEND = 100;
  /* Infinite loop */
  for(;;)
  { 
		// подключение установлено
		if (hello == 1 && book_load  == 0 && param_load == 0 && exper_run == 0 && request_answer == 1)
		{
		  DataToSendUSB_1[0] = 0x21;
		  DataToSendUSB_1[1] = 0x04;
		  my_crc = crc_16_ibm(DataToSendUSB_1,2);
		  DataToSendUSB_1[2] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_1[3] = (uint8_t)(my_crc);
		
		  CDC_Transmit_FS(DataToSendUSB_1, 4);
			request_answer = 0;
			osDelay(PERIOD_TO_SEND);
		}
		// сценарий загружен
		if (book_load == 1 && param_load == 0 && exper_run == 0 && request_answer == 1)
		{
			DataToSendUSB_1[0] = 0xAA;
		  DataToSendUSB_1[1] = 0x04;
		  my_crc = crc_16_ibm(DataToSendUSB_1,2);
		  DataToSendUSB_1[2] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_1[3] = (uint8_t)(my_crc);
		
		  CDC_Transmit_FS(DataToSendUSB_1, 4);
			request_answer = 0;
			osDelay(PERIOD_TO_SEND);
		}
		if (hello == 1 && request_data == 1)
		{
			//Заполнение отправляемых данных
			// если запущен аварийный режим то шлем 0xDF
			if (flag_emergency_mode == 0) {
				DataToSendUSB_1[0] = 0xDD;
				DataToSendUSB_1[1] = 0x0E;
			  DataToSendUSB_1[2] = (uint8_t)(exper_time >> 8);
		    DataToSendUSB_1[3] = (uint8_t)(exper_time);
			}
			else {
				DataToSendUSB_1[0] = 0xDF;
				DataToSendUSB_1[1] = 0x0E;
			  DataToSendUSB_1[2] = (uint8_t)(time_emergency_mode >> 8);
		    DataToSendUSB_1[3] = (uint8_t)(time_emergency_mode);
			}
			DataToSendUSB_1[4] = (uint8_t)(pressure >> 8);
			DataToSendUSB_1[5] = (uint8_t)(pressure);
			//DataToSendUSB_1[4] = (uint8_t)(test_i);
			//test_i = test_i + 1;
		  DataToSendUSB_1[6] = (uint8_t)(moment >> 8);
		  DataToSendUSB_1[7] = (uint8_t)(moment);
//			// отправка первой части сообщения
//			CDC_Transmit_FS(DataToSendUSB_1, 8);
//			// задержка, чтобы контроллер успел отправить сообщение
//			osDelay(30);
			// Статус клапанов
			key_read[0] = HAL_GPIO_ReadPin(Klapan_1);
			key_read[1] = HAL_GPIO_ReadPin(Klapan_2);
			key_read[2] = HAL_GPIO_ReadPin(Klapan_3);
			key_read[3] = HAL_GPIO_ReadPin(Klapan_4);
			key_read[4] = HAL_GPIO_ReadPin(Klapan_5);
			key_read[5] = HAL_GPIO_ReadPin(Klapan_6);
			//сначала обнуляем байт
			DataToSendUSB_2[0] = 0;
			for (i=0;i<6;i++)
		  {
			 //если ключ открыт записать единицу, закрыт 0
			 if (key_read[i] == 1) bitSet(DataToSendUSB_2[0],i);
			 else bitClear(DataToSendUSB_2[0],i);
		  }
			// Статус двигателя 1
			Engine_1_State = HAL_GPIO_ReadPin(Engine_1);
			//сначала обнуляем байт
			DataToSendUSB_2[1] = 0;
		  // заполнение
			//если двигатель включен записать единицу, закрыт 0
			if (Engine_1_State == 1) bitSet(DataToSendUSB_2[1],0);
			else bitClear(DataToSendUSB_2[1],0);
			// двигатель 2
			if (Engine_2_StateStart == 1) bitSet(DataToSendUSB_2[1],1);
			else bitClear(DataToSendUSB_2[1],1);
			// двигатель 2 реверс
			if (Engine_2_StateReverse == 1) bitSet(DataToSendUSB_2[1],2);
			else bitClear(DataToSendUSB_2[1],2);
			// Связь c ZETLAB отсутствует
			if (zetlab_not_connected == 1) bitSet(DataToSendUSB_2[1],3);
			else bitClear(DataToSendUSB_2[1],3);
			// Связь c регулятором давления отсутствует отсутствует
			if (pressure_not_connected == 1) bitSet(DataToSendUSB_2[1],4);
			else bitClear(DataToSendUSB_2[1],4);
			// запись температуры
			DataToSendUSB_2[2] = (uint8_t)(temperature);
			// Запись уровня жидкости
			DataToSendUSB_2[3] = (uint8_t)(liquid_level);
			// получим общий массив для расчета контрольной суммы
			for (i=0;i<16;i++)
		  { if (i<8) DataToSendUSB_all[i] = DataToSendUSB_1[i];
				else if (i>=8) DataToSendUSB_all[i] = DataToSendUSB_2[i-8];
			}
			// Контрольная сумма
			my_crc = crc_16_ibm(DataToSendUSB_all,DataToSendUSB_1[1]-2);
//		  DataToSendUSB_2[4] = (uint8_t)(my_crc >> 8);
//		  DataToSendUSB_2[5] = (uint8_t)(my_crc);
			// отправка сообщения
//			TransmitStatusUSB = CDC_Transmit_FS(DataToSendUSB_all, DataToSendUSB_1[1]-8);
			DataToSendUSB_all[12] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_all[13] = (uint8_t)(my_crc);
			TransmitStatusUSB = CDC_Transmit_FS(DataToSendUSB_all, DataToSendUSB_1[1]);
			request_data = 0;
		}
		// испытание завершено ошибок нет
		if ((exper_run == 3) && (error_status == 0) && flag_operator_stop == 0)
		{
			osDelay(PERIOD_TO_SEND);
			// Эксперимент завершен
			DataToSendUSB_1[0] = 0xEE;
			DataToSendUSB_1[1] = 0x04;
			// Контрольная сумма
			my_crc = crc_16_ibm(DataToSendUSB_1,DataToSendUSB_1[1] - 2);
		  DataToSendUSB_1[2] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_1[3] = (uint8_t)(my_crc);
			// отправка сообщения
			CDC_Transmit_FS(DataToSendUSB_1, DataToSendUSB_1[1]);	
			osDelay(PERIOD_TO_SEND);
			exper_run = 0;
		}
		if (flag_operator_stop == 1){
			exper_run = 0;
			osDelay(PERIOD_TO_SEND);
			// Эксперимент завершен по команде оператора
			DataToSendUSB_1[0] = 0xE1;
			DataToSendUSB_1[1] = 0x04;
			// Контрольная сумма
			my_crc = crc_16_ibm(DataToSendUSB_1,DataToSendUSB_1[1] - 2);
		  DataToSendUSB_1[2] = (uint8_t)(my_crc >> 8);
		  DataToSendUSB_1[3] = (uint8_t)(my_crc);
			// отправка сообщения
			CDC_Transmit_FS(DataToSendUSB_1, DataToSendUSB_1[1]);	
			osDelay(PERIOD_TO_SEND);
			//включаем передачу данных после отправки команды о выключении
      hello = 1;
		  flag_operator_stop = 0;
		}
		osDelay(PERIOD_TO_SEND);
	}  /* USER CODE END Read_ADC */
}
void Read_USB(void const * argument) 
{
  /* USER CODE BEGIN Read_USB */
	uint8_t clear_i = 0;
  /* Infinite loop */
  for(;;)
  { 
		//считывание данных по USB
		if (receive_usb_total>=2)
		{
			new_message_flag = Check_crc_16(received_usb_data,receive_usb_total);
		}
		
		if (new_message_flag == 1)
		{
			//принято приветствие
			if (received_usb_data[0] == 0x21 && received_usb_data[1] == 0x04)
			{
				//Обновление статусов
				book_load  = 0;
				param_load = 0;
				exper_run = 0;
				hello = 1;
				//отправка ответа
				request_answer = 1;
				// очищение буфера
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;				
			}
			//принят сценарий
			if (received_usb_data[0] == 0xCC)
			{
				//очистка сценария
				for (i=0;i<200;i++)
		    {
					book_time[i] = 0; 
					book_pressure[i] = 0;
					book_valve_state[i] = 0;
					book_engine_state[i] = 0;
				}
        //запись сценария
				// первый байт страший
				book_len_p1 = (uint16_t)(received_usb_data[1]*256);
				// второй байт 
				// вычисляем длину сценария 5 = 1+2+2 = номер команды + длина сообщения + CRC
				// делим на 6 т.к. 1 строка это 6 байта (2 время, 2 давление, 1 состояние клапанов, 1 состояние двигателя)			
				book_len_p2 = (uint16_t)(received_usb_data[2]-5);
				book_len = roundf((float)(book_len_p1 + book_len_p2)/6.0);
				for (i=0;i<book_len;i++)
		    {
					// +3 = номер комнады + длина сообщения, i*2 т.к. два байта
					book_time[i] = ((received_usb_data[(i*2)+3]) * 256) + (received_usb_data[(i*2)+3+1]); //склейка
					//пропускаем первые три байта (1=номер команды + 2=длина сообщения) и массив времени
					book_pressure[i] = ((received_usb_data[(i*2)+(book_len*2)+3])*256)+(received_usb_data[(i*2)+(book_len*2)+3+1]);
					//пропускаем первые три байта, массив времени и массив давления
					book_valve_state[i] = (uint8_t)(received_usb_data[(i)+(book_len*4)+3]);
					//пропускаем первые три байта, массив времени и массив давления и массив состояний клапанов
					book_engine_state[i] = (uint8_t)(received_usb_data[(i)+(book_len*5)+3]);
				}
				// обновление параметров
				book_load = 1;
				param_load = 0;
				exper_run = 0;
				request_answer = 1;
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;				
			}
			// принята команда запроса данных
			if (received_usb_data[0] == 0xBF && received_usb_data[1] == 0x04)
			{
				hearth_bit_answer ++;
				request_data = 1;
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;				
			}
			// принята команда старт испытания 1
			if (received_usb_data[0] == 0xBB && received_usb_data[1] == 0x04 && flag_emergency_mode == 0)
			{
				exper_run = 1;
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;				
			}
			// принята команда на слив
			if (received_usb_data[0] == 0xB1 && received_usb_data[1] == 0x04)
			{
				Engine_2_Command = -1;
				exper_run = 0;
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;				
			}
			// принята команда остановки
			if (received_usb_data[0] == 0xEE && received_usb_data[1] == 0x04)
			{
				Engine_2_Command = 0;
				exper_run = 0;
				//все выключаем
				HAL_GPIO_WritePin(Klapan_1_off);
				HAL_GPIO_WritePin(Klapan_2_off);
				HAL_GPIO_WritePin(Klapan_3_off);
				HAL_GPIO_WritePin(Klapan_4_off);
				HAL_GPIO_WritePin(Klapan_5_off);
				HAL_GPIO_WritePin(Klapan_6_off);
			  //Отправка давления в регулятор давления = 0
			  Send_Pressure_RS232(0);
			  //выключение ЭД1 и ЭД2
			  HAL_GPIO_WritePin(Engine_1_off);
				HAL_GPIO_WritePin(Engine_2_start_off);
				HAL_GPIO_WritePin(Engine_2_reverse_off);
				
        ClearBuffer(received_usb_data,&receive_usb_total);
		    receive_usb_total = 0;
		    clear_i = 0;
        osDelay(100);
				// Флаг остановки по команде оператора
				flag_operator_stop = 1;
			}
			// сброс флага принятого сообщения
		new_message_flag = 0;
		}
		//обнуление буфера раз в секунду
		if (clear_i >= 100)
		{			
		ClearBuffer(received_usb_data,&receive_usb_total);
		receive_usb_total = 0;
		clear_i = 0;
		}
		clear_i ++;
		osDelay(10);
	}  /* USER CODE END Read_USB */
}
void Send_UART1(void const * argument)
{
  /* USER CODE BEGIN Send_UART1 */
  /* Infinite loop */
	uint8_t rx_test_bytes[64];
  for(;;)
  {
    xStatus_Tx_UART1 = xQueueReceive(UART1_Tx_Que, &tx1_buffer_go, portMAX_DELAY );	
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
	  TransmitStatusUART1 = HAL_UART_Transmit(&huart1,(uint8_t*)tx1_buffer_go, len_tx1_buffer, 10);
	  if (TransmitStatusUART1 == HAL_OK)
		{
			ClearBuffer(tx1_buffer_go,&len_tx1_buffer);
	  }
	// А С НОВОЙ МИКРУХОЙ БЛЯТЬ РАБОТАЕТ !!!!!!!!!
	// СВЯТАЯ ЗАДЕРЖКА в 3 мс, без нее ZETLAB не работает
	// но и с ней херово работает, не принимает первый байт, адрес устройства и ПОЭТОМУ ПРИМАЕМ ПО 8 БАЙТ
	// его дописываем руками в приеме
	//osDelay(3);	
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);	
//	osDelay(50);
//	HAL_UART_Receive(&huart1,(uint8_t*)&rx1_byte,9,100);
//	osDelay(50);
	  HAL_UART_Receive_IT (&huart1,(uint8_t*)&rx1_byte,9);		
  }
  /* USER CODE END Send_UART1 */
}
void Send_UART2(void const * argument)
{
  /* USER CODE BEGIN Send_UART2 */
  /* Infinite loop */
  for(;;)
  {	
		//	osDelay(50);
//	HAL_UART_Receive(&huart2,(uint8_t*)&rx2_byte,4,100);
//	osDelay(50);
	  HAL_UART_Receive_IT (&huart2,rx2_byte,4);
    xStatus_Tx_UART2 = xQueueReceive(UART2_Tx_Que, &tx2_buffer_go, portMAX_DELAY );
	  TransmitStatusUART2 = HAL_UART_Transmit(&huart2,(uint8_t*)tx2_buffer_go, len_tx2_buffer, 10);
	  if (TransmitStatusUART2 == HAL_OK){
		  ClearBuffer(tx2_buffer_go,&len_tx2_buffer);
	  }
  }
  /* USER CODE END Send_UART2 */
}
void ReceiveUART1(void const * argument)
{
  /* USER CODE BEGIN ReceiveCommand */
	uint8_t rx1_byte_rtos[9];
	uint8_t uart1_i = 0;
	//uint8_t rx2_byte_rtos[4];
	//uint8_t uart2_i = 0;
  /* Infinite loop */
  for(;;)
  {
		xStatus_Rx_UART1 = xQueueReceive(UART1_Rx_Que, &rx1_byte_rtos, portMAX_DELAY );
		//считываем момент
		//так как первый байт не принимаем, пишем его руками и ПОЭТОМУ ПРИМАЕМ ПО 8 БАЙТ
		//rx1_message[0] = 0x05;
		for (uart1_i=0;uart1_i<=sizeof(rx1_byte_rtos)/sizeof(rx1_byte_rtos[0]);uart1_i++ ){
		// +1 так как первый байт не принимаем
		//rx1_message[uart1_i+1] = rx1_byte_rtos[uart1_i];
			rx1_message[uart1_i] = rx1_byte_rtos[uart1_i];
		}
		rx1_message_count=9;
		new_message_flag_uart = Check_crc_16_modbus(rx1_message,rx1_message_count);
//		// А это надо так как иногда мы принимаем с третьего байта а не со второго
//		// два байта пишем ручками
//		if (new_message_flag_uart != 1){
//		//так как первый байт не принимаем, пишем его руками
//		rx1_message[0] = 0x05;
//		rx1_message[1] = 0x03;
//		for (uart1_i=0;uart1_i<=sizeof(rx1_byte_rtos)/sizeof(rx1_byte_rtos[0]);uart1_i++ ){
//		// +1 так как первый байт не принимаем
//		rx1_message[uart1_i+2] = rx1_byte_rtos[uart1_i];
//		}
//		rx1_message_count=9;
//		new_message_flag_uart = Check_crc_16_modbus(rx1_message,rx1_message_count);
//		}
	  if (new_message_flag_uart == 1)
		{
			zetlab_wait_answer = 0;
			// D6 6D 42 9E
		 //bytes = {0x6D,0xD6,0x9E,0x42};
			bytes[0]=rx1_message[4];
			bytes[1]=rx1_message[3];
			bytes[2]=rx1_message[6];
			bytes[3]=rx1_message[5];
			moment_float =*(float *)&bytes;
			// умножаем на 2, чтоб была точность 0.5,
			if (moment_float < 0) moment = 0;
			else moment = roundf((moment_float)*2);
      new_message_flag_uart = 0;
	  }

		for (uart1_i=0;uart1_i<=sizeof(rx1_byte_rtos)/sizeof(rx1_byte_rtos[0]);uart1_i++ ){
		rx1_message[uart1_i] = 0;
		rx1_byte_rtos[uart1_i] = 0;	
		}
//		rx1_message_count=9;
//		new_message_flag_uart = Check_crc_16_modbus(rx1_byte,rx1_message_count);
//	  if (new_message_flag_uart == 1)
//		{
//			zetlab_wait_answer = 0;
//			// D6 6D 42 9E
//		 //bytes = {0x6D,0xD6,0x9E,0x42};
//			bytes[0]=rx1_byte[4];
//			bytes[1]=rx1_byte[3];
//			bytes[2]=rx1_byte[6];
//			bytes[3]=rx1_byte[5];
//			moment_float =*(float *)&bytes;
//			// умножаем на 2, чтоб была точность 0.5, -0.4 - калибровка
//			moment = roundf((moment_float-0.4)*2);
//      new_message_flag_uart = 0;
//	  }
//		else if (rx2_byte[0] == 0x04 && rx2_byte[1] == 0xBF) {
//			pressure_wait_answer = 0;
//			pressure=((uint16_t)rx2_byte[2] << 8) | rx2_byte[3];
//		}
//		for (uart_i=0;uart_i<=sizeof(rx1_byte)/sizeof(rx1_byte[0]);uart_i++ ){
//		rx1_byte[uart_i] = 0;	
//		}
//		for (uart_i=0;uart_i<=sizeof(rx2_byte)/sizeof(rx2_byte[0]);uart_i++ ){
//		rx2_byte[uart_i] = 0;	
//		}
//		osDelay(100);
  }
  /* USER CODE END ReceiveCommand */
}
void ReceiveUART2(void const * argument)
{
  /* USER CODE BEGIN ReceiveCommand */
	uint8_t rx2_byte_rtos[4];
//	uint8_t uart2_i = 0;
	//uint8_t rx2_byte_rtos[4];
	//uint8_t uart2_i = 0;
  /* Infinite loop */
  for(;;)
  {
		xStatus_Rx_UART2 = xQueueReceive(UART2_Rx_Que, &rx2_byte_rtos, portMAX_DELAY );
		//Считываем давление
		if (rx2_byte_rtos[0] == 0x04 && rx2_byte_rtos[1] == 0xBF) {
			pressure_not_connected = 0;
			pressure_wait_answer = 0;
			pressure=((uint16_t)rx2_byte_rtos[2] << 8) | rx2_byte_rtos[3];
		}
  }
  /* USER CODE END ReceiveCommand */
}
uint16_t cl_i =0;
void ClearBuffer(uint8_t *buffer,uint16_t *len_buffer)
{ 
	cl_i = 0;
	for(cl_i=0;cl_i<=*len_buffer;cl_i++){
		buffer[cl_i] = 0;
	}
	*len_buffer = 0;
}
void Send_Pressure_RS232(uint16_t value){
	p_value_rs232 = (uint16_t)(value);
	//установка требуемого давления
	tx2_buffer_que[0] = 0x04;
	tx2_buffer_que[1] = 0x21;
	//tx2_buffer_que[1] = 0xBF;
	tx2_buffer_que[2] = (uint8_t)(p_value_rs232 >> 8);
  tx2_buffer_que[3] = (uint8_t)(p_value_rs232);
	len_tx2_buffer = 4;
	//включаем прием сообщения
	//HAL_UART_Receive_IT (&huart2,(uint8_t*)&rx2_byte,4);
	// отправляем запрос давления
	xStatus_Tx_UART2 = xQueueSendToBack(UART2_Tx_Que,&tx2_buffer_que,0);
}
uint16_t crc_16_ibm(uint8_t *buf, uint16_t len) 
{
  uint16_t crc = 0;
  uint16_t j;
  while (len--) {
    crc ^= *buf++;
    for (j = 0; j < 8; j++)
		{
			crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
		}
  }
return crc;
}
uint16_t CRC16_2(unsigned char *buf, int len)
{  
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
  crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

  for (int i = 8; i != 0; i--) {    // Loop over each bit
    if ((crc & 0x0001) != 0) {      // If the LSB is set
      crc >>= 1;                    // Shift right and XOR 0xA001
      crc ^= 0xA001;
    }
    else                            // Else LSB is not set
      crc >>= 1;                    // Just shift right
    }
  }

  return crc;
}
uint16_t control_crc16 = 0;
uint8_t Check_crc_16(uint8_t *buf, uint16_t len)
{
	 control_crc16 = crc_16_ibm(buf, len-2);
	 if (buf[len-1] == (uint8_t)(control_crc16) && 
		   buf[len-2] == (uint8_t)(control_crc16 >> 8))
	 {
		 return 1;
	 }
	 else return 0;
}
uint8_t Check_crc_16_modbus(uint8_t *buf, uint16_t len)
{
	 control_crc16 = CRC16_2(buf, len-2);
	 if (buf[len-2] == (uint8_t)(control_crc16) && 
		   buf[len-1] == (uint8_t)(control_crc16 >> 8))
	 {
		 return 1;
	 }
	 else return 0;
}
void Control_Engine_2(int8_t command, uint8_t Klapan_6_Readed)
{
	if (Klapan_6_Readed == 1 && command == 1) HAL_GPIO_WritePin(Klapan_6_on);
	else HAL_GPIO_WritePin(Klapan_6_off);
	// остановка
	if (command == 0 && Engine_2_StateStop == 0){
	  // закрытие КЭ6
	  HAL_GPIO_WritePin(Klapan_6_off);
	  //включение ЭД2
	  HAL_GPIO_WritePin(Engine_2_stop_on);
	  osDelay(100);
	  HAL_GPIO_WritePin(Engine_2_stop_off);
	  // Статус запуска двигателя 2
	  Engine_2_StateStart = 0;
	  // Статус остановки двигателя 2 
	  Engine_2_StateStop = 1;
	  // Статус реверса двигателя 2
	  Engine_2_StateReverse = 0;
	}
	if (command == 1 && Engine_2_StateStart == 0){
	  //заполнение жидкостью 			
	  //включение ЭД2
	  HAL_GPIO_WritePin(Engine_2_start_on);
	  osDelay(100);
	  HAL_GPIO_WritePin(Engine_2_start_off);
	  // Статус запуска двигателя 2
	  Engine_2_StateStart = 1;
	  // Статус остановки двигателя 2 
	  Engine_2_StateStop = 0;
	  // Статус реверса двигателя 2
	  Engine_2_StateReverse = 0;	
	}
	if (command == -1 && Engine_2_StateReverse == 0){
	//откачка жидкости			
	HAL_GPIO_WritePin(Klapan_3_on);
	HAL_GPIO_WritePin(Klapan_4_on);
	HAL_GPIO_WritePin(Klapan_5_on);
	//включение ЭД2 в режиме
	HAL_GPIO_WritePin(Engine_2_reverse_on);
	osDelay(100);
	HAL_GPIO_WritePin(Engine_2_reverse_off);
	// Статус запуска двигателя 2
	Engine_2_StateStart = 0;
	// Статус остановки двигателя 2 
	Engine_2_StateStop = 0;
	// Статус реверса двигателя 2
	Engine_2_StateReverse = 1;	
	}
}

void My_Delay(uint32_t time_delay)
{
	// Тут есть отключение 2 и 5 клапана
	uint16_t period_delay = 200;
	uint32_t count_delay = 0;
	for (count_delay=0;count_delay<time_delay;count_delay=count_delay+period_delay){
		if (exper_run == 0 || flag_emergency_mode==1) break;
		else {
			// состояние клапана 2 и 5
			if (i_exper <= 1 && liquid_level >= h_max) {
				HAL_GPIO_WritePin(Klapan_2_off);
				HAL_GPIO_WritePin(Klapan_5_off);
			}
			osDelay(period_delay);
		}
	}
}
void My_Emergency(uint32_t time_delay)
{
	uint16_t period_delay = 200;
	uint32_t count_delay = 0;
	for (count_delay=0;count_delay<time_delay;count_delay=count_delay+period_delay){
			osDelay(period_delay);
			time_emergency_mode = time_emergency_mode - 2;
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
