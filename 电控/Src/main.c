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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_uart.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
	noaction = 0,				//什么也不做
	manualstart,				//手动触碰，一定时间后松开
	ready,							//读取数据，准备放下
	touch,							//放下竹签，触碰屏幕
	jumping,						//抬起
}jump_state_t;
jump_state_t jump_state;

uint32_t manual_touch_time = 750;
uint32_t jump_time;
uint32_t wait_time = 0;

int32_t totalecd_target;
int16_t speedtar_3508;
int16_t current_3508;


#define touch_ecd		2000
#define origin_ecd -2000

float param_a = 1;			//1900
float param_b = 0;
float measure_distance;

extern float rece_distance_midvalue;

//#define origin_ccr	1220		//1220
//#define touch_ccr		1100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	{
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
	dbus_init();
	manifold_uart_init();
	}	
	
	
	my_can_filter_init_recv_all();
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0); //放后面会死机
	
	HAL_Delay(3000);
	
	
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // test
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
//	ReadAllFromFlash();
	PID_struct_init(&pid_3508,			 POSITION_PID, 5000, 2000, 0.3f, 0.0f,  0);
	PID_struct_init(&pid_3508_speed, POSITION_PID, 6000, 2000, 6, 	0.0f,  	0);

	totalecd_target = origin_ecd;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		static uint32_t touch_start_tick;
		static uint32_t jump_start_tick;
		static uint8_t last_rc_sw1 = 0;
		static uint8_t last_rc_sw2 = 0;
		uint32_t now = HAL_GetTick();
		
		if(rc.sw1 == RC_MI)
			jump_state = noaction;
			
		
		switch(jump_state)
		{
			case noaction:
			{
				/* 右拨杆打上之后可用ch2摇杆控制位置，打中归零 */
				if(rc.sw2 == RC_UP)
				{
					totalecd_target += rc.ch2;
				}

				else
				{
					if(rc.sw2 == RC_MI && last_rc_sw2 == RC_UP)		
					{
						moto_ecd_reset(&moto_3508);
					}
					
					totalecd_target = origin_ecd;
					if(rc.sw1 == RC_DN)
					{
						jump_state = ready;
					}
					else if(rc.sw1 == RC_UP && last_rc_sw1 != RC_UP)
					{
						jump_state = manualstart;
						touch_start_tick = now;
					}
				}
			}
			break;
				
			case manualstart:
			{
				totalecd_target = touch_ecd;
				if(now - touch_start_tick > manual_touch_time)
				{
					jump_state = noaction;								//release the steering
				}
			}
			break;
			
			case ready:
			{
				totalecd_target = origin_ecd;
				if(rece_data.sofa == 'a' && rece_data.sof5 == '5' && rece_data.endf1 == 'f' && rece_data.endf2 == 'f' \
					 && rece_flag == 1)
				{
					jump_time = rece_data.time;				//获取应该按下的触摸时长
					touch_start_tick = now;
					jump_state = touch;							//put down the touch thing
				}
			}
			break;
				
			case touch:
			{
				totalecd_target = touch_ecd;
				if(now - touch_start_tick > jump_time)
				{
					jump_state = jumping;								//release the 3508
					jump_start_tick = now;
				}
			}
			break;
				
			case jumping:
			{
				totalecd_target = origin_ecd;
				rece_flag = 0;							//抬起后，将接收标志置0.标志在接收中断中复位
				if(now - jump_start_tick > wait_time)
				{
					jump_state = ready;
				}
			}
			break;
		}
		
//		TIM2->CCR1 = rc.ch4 + 1500;
		last_rc_sw1 = rc.sw1;
		last_rc_sw2 = rc.sw2;
		
		
//		speedtar_3508 = rc.ch4 * 5;
//		totalecd_target += rc.ch4 / 10;
		speedtar_3508 = pid_calc(&pid_3508, moto_3508.total_ecd, totalecd_target);
		current_3508 = pid_calc(&pid_3508_speed, moto_3508.speed_rpm, speedtar_3508);
		
		set_200_current(&hcan1, current_3508, 0, 0, 0);
		HAL_Delay(2);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

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
