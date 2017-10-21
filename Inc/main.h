/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IR_Sensor_Pin GPIO_PIN_4
#define IR_Sensor_GPIO_Port GPIOA
#define LS_TOP_Pin GPIO_PIN_7
#define LS_TOP_GPIO_Port GPIOA
#define LS_BUTTOM_Pin GPIO_PIN_0
#define LS_BUTTOM_GPIO_Port GPIOB
#define RS485_SIGNAL_Pin GPIO_PIN_2
#define RS485_SIGNAL_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_3
#define M1_GPIO_Port GPIOB
#define M2_Pin GPIO_PIN_4
#define M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define POSITION_0_X 470
#define POSITION_0_Y 45

#define POSITION_1_X (POSITION_0_X+0)
#define POSITION_1_Y (POSITION_0_Y+160)

#define POSITION_2_X (POSITION_0_X+0)
#define POSITION_2_Y (POSITION_0_Y+320)

#define POSITION_3_X (POSITION_0_X+0)
#define POSITION_3_Y (POSITION_0_Y+480)

#define POSITION_4_X (POSITION_0_X+0)
#define POSITION_4_Y (POSITION_0_Y+540)

#define POSITION_5_X (POSITION_0_X-400)
#define POSITION_5_Y (POSITION_0_Y+540)

#define POSITION_6_X (POSITION_0_X-400)
#define POSITION_6_Y (POSITION_0_Y+480)

#define POSITION_7_X (POSITION_0_X-400)
#define POSITION_7_Y (POSITION_0_Y+320)

#define POSITION_8_X (POSITION_0_X-400)
#define POSITION_8_Y (POSITION_0_Y+160)

#define POSITION_UD_X 0
#define POSITION_UD_Y 0

#define DELTA_X 28

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
