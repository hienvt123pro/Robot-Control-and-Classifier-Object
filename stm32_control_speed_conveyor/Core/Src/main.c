/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const int MAX_LEN = 5;
const int NEW_RECIEVE = 11;
const int OLD_RECIEVE = 22;
uint8_t oldRxData[22] = {0};  
uint8_t newRxData[11] = {0};  
uint8_t nTxData[5] = {0};

uint8_t reStart[] = {0x00};
uint8_t reStop[] = {0x00};
uint8_t reDevice[] = {0x00};
uint8_t reCmd[2] = {0};
uint8_t reData[6] = {0};
uint8_t reComma[] = {0x00};
uint8_t sendData[2] = {0};   
uint8_t sendStart[] = {0x30};
uint8_t sendStop[] = {0x31};
uint8_t sendDevice[] = {0x32};
uint8_t Comma[] = {0x2C};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int enc = 0;
double speed = 0;
double v_speed = 0;
double Kp =950, Ki=830, Kd =320;

int duty=0;
double e, ep, p_e=0, pp_e=0, pPart=0 , iPart=0, dPart=0, udk, predPart = 0, pwm, intergralspeed, pre_e;
int m_i = 0;
double setpoint = 2,acc_for_pos,vel_for_pos;
double speed_kalman,temp5,position_kalman,sp_kalman,pos_kalman,mi_kalman;
int  run=0, mode=0;
float dErrorTerm1, dErrorTerm2;
float alpha, beta, gamma, delta, dPIDResult, dPIDResultTerm; 
bool flagStart = false;
bool DataAvailable = false;
int i = 0;
bool isSizeError = false;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t ENC0DER_GetSpeed(void);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_speed(double act);
void SubFrame(void);
void WriteComm(uint8_t *pBuff, uint8_t nSize);
void ReadComm(uint8_t *pBuff, uint8_t nSize);
void subString(uint8_t *s, uint8_t *t,int start, int end);
void removeComma(uint8_t *oldFrame, uint8_t *newFrame, int start, int end);



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)oldRxData, sizeof(oldRxData));
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		removeComma(oldRxData, newRxData, 0, 20);
    ReadComm(newRxData,11);
		if (DataAvailable == true)
		{
			if (reCmd[0] == 0x37 && reCmd[1] == 0x30 && reDevice[0] == 0x32) 
			{ 
				pp_e = 3;
				flagStart = true;
				setpoint = (double)(reData[0]-0x30);
			}
			else if (reCmd[0] == 0x37 && reCmd[1] == 0x31 && reDevice[0] == 0x32)
			{ 
				pp_e = 4;
				flagStart = false;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);			
			}		
			else if (reCmd[0] == 0x38 && reCmd[1] == 0x30 && reDevice[0] == 0x32)
			{
				pp_e = 5; 
				double pre_v_speed = 0 ;
				sendData[0] = floor(v_speed);
				pre_v_speed = (v_speed - floor(v_speed))* 10 ;
				sendData[1] = floor(pre_v_speed);
				SubFrame();
			}
			else if (reCmd[0] == 0x39 && reCmd[1] == 0x30 && reDevice[0] == 0x32)
			{
				isSizeError = true;
				if (i == 0) {
					i = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				}
				if (i == 1) {
					i = 0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				}	
			}
			DataAvailable = false;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim2.Instance)
 { 
	 if (flagStart == true)
	 { enc = (int32_t)__HAL_TIM_GetCounter(&htim4);
	 if (enc  > 64000 )
	 {  enc = 65535 - enc;}
	 speed = (enc*60)/0.05/2400;
   v_speed = (speed * 3.14159 * 3.5) / 60;
	 PID_speed(v_speed); 

  	__HAL_TIM_SET_COUNTER(&htim4, 0);
   }
 }
}



void PID_speed(double act)
{ 
	float dTs = 0.05f;
e = setpoint - act;
alpha = 2*dTs*Kp + Ki*dTs*dTs + 2*Kd;
beta = Ki*dTs*dTs - 4*Kd - 2*dTs*Kp;
gamma = 2*Kd;
 delta = 2*dTs;
dPIDResult = (alpha*e + beta*dErrorTerm1 + gamma*dErrorTerm2 + delta*dPIDResultTerm)/delta;
dPIDResultTerm = dPIDResult;
dErrorTerm2 = dErrorTerm1;
dErrorTerm1 = e;
//udk = dPIDResult;

 if ((int)dPIDResult > 7199)
    { pwm = 7199;
		dPIDResult = 7199;}
		   
 else if (dPIDResult < 0)
 { pwm = -dPIDResult;
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
 }
 else if ((int)dPIDResult < -7199)
	 pwm = 7199;
 else 
 {pwm = dPIDResult;
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
 }

   duty=(int)pwm;
 	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,duty);
}


void SubFrame(void) 
{
	    int nIndex = 0;
     	memcpy(nTxData + nIndex,sendStart,1);
			nIndex += 1;
			memcpy(nTxData + nIndex,sendDevice,1);
			nIndex += 1;
			memcpy(nTxData + nIndex,sendData,2);
			nIndex += 2;
			memcpy(nTxData + nIndex,sendStop,1);
			WriteComm(nTxData, 5);
}

void WriteComm(uint8_t *pBuff, uint8_t nSize){
	   
	 HAL_UART_Transmit(&huart1, pBuff, nSize, 1000);
}


//How to Read
/*
  removeComma(oldRxData, newRxData, 0, 20);
  ReadComm(newRxData, RECIEVE);
 
*/

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
////	if(huart->Instance == huart1.Instance)
////	{
//		 //removeComma(oldRxData, newRxData, 0, 20);
//		//ReadComm(newRxData,11);
//	HAL_UART_Receive_IT(&huart1,(uint8_t *)oldRxData, 22);
//	//}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		DataAvailable = true;
	}
}

void ReadComm(uint8_t *pBuff, uint8_t nSize)
{  
	if((pBuff[0] == 0x30) && (pBuff[10]) == 0x31 ) 
	{
		  subString(pBuff,reStart,0,0);
	 	  subString(pBuff,reDevice,1,1);
	    subString(pBuff,reCmd,2,3);
	  	subString(pBuff,reData,4,9);
	  	subString(pBuff,reStop,10,10);
	}

}

void removeComma(uint8_t *oldFrame, uint8_t *newFrame, int start, int end) 
{
	int index = 0;

	for(int i = start; i <= end; i = i+2)  // start 0 , end 20
	{
		newFrame[index] = oldFrame[i];
		index++;
	}
}

void subString(uint8_t *s, uint8_t *t,int start, int end)
{
	int index = 0;
	for(int i = start; i <= end; i++)
	{
		t[index] = s[i];
		index++;
	}
}
	
	

/* EXTI */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == rise_edge_sensor_Pin)
	{
		if (isSizeError == true) {
			if (i == 0) {
				i = 1;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			}
			if (i == 1) {
				i = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			}
			isSizeError = false;
		}
	}
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

