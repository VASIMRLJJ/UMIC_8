/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "enum.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum Moving_State Updown;
enum CoreXY_Action Updown_Act;
enum Moving_State CoreXY_State;
enum Moving_State Scissors_State;
enum Circle_State Mcircle = Circle_Init;
uint8_t Recive_Data[4];
uint8_t Scissors_Recive_Buffer[30];
uint8_t CoreXY_Recive_Buffer[30];
uint8_t CoreXY_Transmit_Buffer[30];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Block_Clear(uint8_t* buffer);
void Moving_Updown(enum Moving_State state);
void Moving_CoreXY(void);
void CoreXY_Act(uint8_t position, enum CoreXY_Action action);
void CoreXY_Send(enum CoreXY_Command command);
void Mechanic_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	Mechanic_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		//HAL_Delay(200);
		if(Mcircle == Circle_Idle)
			continue;
		if(Updown != Moving && Updown_Act == Stop)
		{
			HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_RESET);
			if(Updown == At_Buttom)
				Mcircle = Circle_Idle;
			else
			{
				Moving_CoreXY();
				Moving_Updown(At_Top);
			}
		}
		else
		{
			if(HAL_GPIO_ReadPin(LS_BUTTOM_GPIO_Port, LS_BUTTOM_Pin) == GPIO_PIN_RESET
				|| HAL_GPIO_ReadPin(LS_TOP_GPIO_Port, LS_TOP_Pin) == GPIO_PIN_RESET)
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(LS_BUTTOM_GPIO_Port, LS_BUTTOM_Pin) == GPIO_PIN_RESET && Updown_Act == Down)
				{
					Updown = At_Buttom;
					Updown_Act = Stop;
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				}
				if(HAL_GPIO_ReadPin(LS_TOP_GPIO_Port, LS_TOP_Pin) == GPIO_PIN_RESET && Updown_Act == Up)
				{
					Updown = At_Top;
					Updown_Act = Stop;
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
				}
			}
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_SIGNAL_Pin|GPIO_PIN_12|M1_Pin|M2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_Sensor_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_TOP_Pin */
  GPIO_InitStruct.Pin = LS_TOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LS_TOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_BUTTOM_Pin */
  GPIO_InitStruct.Pin = LS_BUTTOM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LS_BUTTOM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_SIGNAL_Pin PB12 */
  GPIO_InitStruct.Pin = RS485_SIGNAL_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_Pin M2_Pin */
  GPIO_InitStruct.Pin = M1_Pin|M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  USART recive interrupt callback function.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		switch(Mcircle)
		{
			case Circle_Idle: 
				Mcircle = Circle_Moving;
				HAL_UART_Transmit(&huart2, (uint8_t*)"OK", sizeof("OK"), 1000);
				Moving_Updown(At_Buttom);
				break;
			default: 
				HAL_UART_Transmit(&huart2, (uint8_t*)"BUSY", sizeof("BUSY"), 1000);
		}
	}
	if(huart == &huart1)
	{
		if(!memcmp(CoreXY_Recive_Buffer, "ok\n", sizeof("ok\n")))
			CoreXY_State = Finished;
		else
			HAL_UART_Transmit(&huart2, (uint8_t*)"COREXY ERROR", sizeof("COREXY ERROR"), 1000);
	}
		if(huart == &huart3)
	{
		if(!memcmp(CoreXY_Recive_Buffer, "DONE", sizeof("DONE")))
			Scissors_State = Finished;
		else
			HAL_UART_Transmit(&huart2, (uint8_t*)"COREXYERROR", sizeof("COREXYERROR"), 1000);
	}
}

/**
  * @brief USART transmit interrupt callback function.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		Block_Clear(CoreXY_Recive_Buffer);
		HAL_UART_Receive_IT(&huart1, CoreXY_Recive_Buffer, sizeof("ok\n"));
	}
	if(huart == &huart3)
	{
		HAL_GPIO_WritePin(RS485_SIGNAL_GPIO_Port, RS485_SIGNAL_Pin, GPIO_PIN_RESET);
		Block_Clear(Scissors_Recive_Buffer);
		HAL_UART_Receive_IT(&huart3, Scissors_Recive_Buffer, sizeof("DONE"));
	}

}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LS_BUTTOM_Pin)
		Updown = At_Buttom;
	else if(GPIO_Pin == LS_TOP_Pin)
		Updown = At_Top;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}*/

/**
  * @brief To reset a block of memory into 0000...
	* @param buffer: A pointer to a block of memory
  * @retval None
  */
void Block_Clear(uint8_t* buffer)
{
	uint8_t i;
	for(i=0; i<sizeof(buffer); i++)
	{
		*(buffer + i) = 0;
	}
}

/**
  * @brief To set a COREXY action
	* @param action: An action type in the CoreXY_Action enum
  * @retval None
  */
void CoreXY_Act(uint8_t position, enum CoreXY_Action action)
{
	unsigned int xy[] = {0, 0};
	switch(position)
	{
		case 0: xy[0] = POSITION_0_X; xy[1] = POSITION_0_Y; break;
		case 1: xy[0] = POSITION_1_X; xy[1] = POSITION_1_Y; break;
		case 2: xy[0] = POSITION_2_X; xy[1] = POSITION_2_Y; break;
		case 3: xy[0] = POSITION_3_X; xy[1] = POSITION_3_Y; break;
		case 4: xy[0] = POSITION_4_X; xy[1] = POSITION_4_Y; break;
		case 5: xy[0] = POSITION_5_X; xy[1] = POSITION_5_Y; break;
		case 6: xy[0] = POSITION_6_X; xy[1] = POSITION_6_Y; break;
		case 7: xy[0] = POSITION_7_X; xy[1] = POSITION_7_Y; break;
		case 8: xy[0] = POSITION_8_X; xy[1] = POSITION_8_Y; break;
		case 9: xy[0] = POSITION_UD_X; xy[1] = POSITION_UD_Y; break;
	}
	if(action == Up)
	{
		uint16_t i;
		Block_Clear(CoreXY_Transmit_Buffer);
		i = sprintf((char*)CoreXY_Transmit_Buffer, "G01 X%d Y%d F150\n", xy[0] - DELTA_X, xy[1]);
		HAL_UART_Transmit(&huart1, CoreXY_Transmit_Buffer, i+1, 1000);
		CoreXY_State = Moving;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)"G4 P0.01", sizeof("G4 P0.01"));
		while(CoreXY_State == Moving);
		HAL_GPIO_WritePin(RS485_SIGNAL_GPIO_Port, RS485_SIGNAL_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)"DOWN", sizeof("DOWN"));
		Scissors_State = Moving;
		while(Scissors_State == Moving);
		
		Block_Clear(CoreXY_Transmit_Buffer);
		i = sprintf((char*)CoreXY_Transmit_Buffer, "G01 X%d Y%d F150\n", xy[0], xy[1]);
		HAL_UART_Transmit(&huart1, CoreXY_Transmit_Buffer, i+1, 1000);
		CoreXY_State = Moving;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)"G4 P0.01", sizeof("G4 P0.01"));
		while(CoreXY_State == Moving);
		HAL_GPIO_WritePin(RS485_SIGNAL_GPIO_Port, RS485_SIGNAL_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)"UP", sizeof("UP"));
		Scissors_State = Moving;
		while(Scissors_State == Moving);
	}
	else
	{
		uint16_t i;
		Block_Clear(CoreXY_Transmit_Buffer);
		i = sprintf((char*)CoreXY_Transmit_Buffer, "G01 X%d Y%d F100\n", xy[0], xy[1]);
		HAL_UART_Transmit(&huart1, CoreXY_Transmit_Buffer, i+1, 1000);
		CoreXY_State = Moving;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)"G4 P0.01", sizeof("G4 P0.01"));
		while(CoreXY_State == Moving);
		HAL_GPIO_WritePin(RS485_SIGNAL_GPIO_Port, RS485_SIGNAL_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)"DOWN", sizeof("DOWN"));
		Scissors_State = Moving;
		while(Scissors_State == Moving);
		
		Block_Clear(CoreXY_Transmit_Buffer);
		i = sprintf((char*)CoreXY_Transmit_Buffer, "G01 X%d Y%d F150\n", xy[0] - DELTA_X, xy[1]);
		HAL_UART_Transmit(&huart1, CoreXY_Transmit_Buffer, i+1, 1000);
		CoreXY_State = Moving;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)"G4 P0.01", sizeof("G4 P0.01"));
		while(CoreXY_State == Moving);
		HAL_GPIO_WritePin(RS485_SIGNAL_GPIO_Port, RS485_SIGNAL_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)"UP", sizeof("UP"));
		Scissors_State = Moving;
		while(Scissors_State == Moving);
	}
}

void CoreXY_Send(enum CoreXY_Command command)
{
	switch(command)
	{
		case CoreXY_Home: HAL_UART_Transmit(&huart1, (uint8_t*)"$H\n", sizeof("$H\n"), 1000);break;
		case CoreXY_TogXY: HAL_UART_Transmit(&huart1, (uint8_t*)"$3=3\n", sizeof("$3=3\n"), 1000);break;
	}
	while(CoreXY_State == Moving);
}

void Mechanic_Init(void)
{
	//CoreXY_Send(CoreXY_TogXY);
	//CoreXY_Send(CoreXY_Home);
	HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_RESET);
	HAL_Delay(3000);
	Moving_Updown(At_Top);
	HAL_UART_Receive_IT(&huart2, Recive_Data, sizeof(Recive_Data));
}

void Moving_Updown(enum Moving_State state)
{
	if(state == Moving)
	{
		return;
	}
	else if(state == At_Top)
	{
		HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_SET);
		Updown_Act = Down;
	}
	else
	{
		HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_RESET);
		Updown_Act = Up;
	}
	Updown = Moving;
}

void Moving_CoreXY(void)
{
	uint8_t put, get;
	put = Recive_Data[0]-'0';
	get = Recive_Data[2]-'0';
	Block_Clear(Recive_Data);
	if(put<=8&&get<=8)
	{
		CoreXY_Act(9, Up);
		CoreXY_Act(put, Down);
		CoreXY_Act(get, Up);
		CoreXY_Act(9, Down);
	}
	else
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"ILIGLE INPUT", sizeof("ILIGLE INPUT"), 1000);
		HAL_UART_Transmit(&huart2, &put, 1, 1000);
		HAL_UART_Transmit(&huart2, &get, 1, 1000);		
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
