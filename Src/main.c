/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author			: 宋裕恒
  * @brief			: 本程序适用于H743\H753开发板，包括光纤惯导、XSense惯导、UWB位置传感器、深度传感器
  * @introduction	: USART6->XSense惯导、  UART7->深度传感器、  UART5->与RCT6通信、  USART2->光纤惯导、  USART1->UWB
  * @attention		:
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "U8_Float.h"
#include "UWB.h"
#include "Xsense.h"
#include "Depth_Sensor.h"
#include "GuangXian.h"
#include "Dvl.h"
#include "Cal_V_Angular.h"
#include "Navigation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 全局变量

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int b = 0;

//以下为实际输出变量声明
DVL_Parser_t dvl_parser;
uint8_t dvl_rx_byte;
DeadReckoning_t dr;
uint32_t last_position_update = 0;
Position_t pos;

Velocity_t body_velocity;
Attitude_t attitude;      
uint32_t current_time;
Position_t current_pos;

float sensor_u, sensor_v, sensor_w;           // 速度数据
float sensor_roll, sensor_pitch, sensor_yaw;  // 姿态数据
float sensor_depth;                           // 深度数据

float Xaxis;
float Yaxis;

float depth_num;
float Angle = 0.0;
float roll = 0.0;
float Pitch;

float speed_u;
float speed_v;
float speed_w;
float speed_p;
float speed_q;
float speed_r;


//定义发送的数组
uint8_t Xpos[4] = {0};//X
uint8_t Ypos[4] = {0};
uint8_t Depth[4] = {0};

uint8_t Roll[4] = {0};
uint8_t pitch[4] = {0};//跟上面float重名了，用小写
uint8_t yaw[4] = {0};

uint8_t u_vlocity[4] = {0};
uint8_t v_vlocity[4] = {0};
uint8_t w_vlocity[4] = {0};

uint8_t p_vlocity[4] = {0};
uint8_t q_vlocity[4] = {0};
uint8_t r_vlocity[4] = {0};


uint8_t Deta[51] = {0};//最后的数据

uint8_t UART6_Rec_Buf[50] = {0};
uint8_t UART7_Rec_Buf[50] = {0};

uint8_t UART6_Rec_Counter = 0;
uint8_t UART7_Rec_Counter = 0;
//深度计全局变量
uint8_t Depth_rx_buffer[1];
char Depth_rx_line[50];
uint8_t Depth_rx_index = 0;
float D_value = 0.0f;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_D_value(void) { return D_value; }
void Navigation_Task(void);
void Fuzhi()
{
  Deta[0] = 0xFE;
	Deta[1] = 0x01;
	
	Deta[2] = Xpos[0];
	Deta[3] = Xpos[1];
	Deta[4] = Xpos[2];
	Deta[5] = Xpos[3];
	
	Deta[6] = Ypos[0];
	Deta[7] = Ypos[1];
	Deta[8] = Ypos[2];
	Deta[9] = Ypos[3];
	
	Deta[10] = Depth[0];
	Deta[11] = Depth[1];
	Deta[12] = Depth[2];
	Deta[13] = Depth[3];
	
	Deta[14] = Roll[0];
	Deta[15] = Roll[1];
	Deta[16] = Roll[2];
	Deta[17] = Roll[3];
	
	Deta[18] = pitch[0];
	Deta[19] = pitch[1];
	Deta[20] = pitch[2];
	Deta[21] = pitch[3];
	
	Deta[22] = yaw[0];
	Deta[23] = yaw[1];
	Deta[24] = yaw[2];
	Deta[25] = yaw[3];
	
	Deta[26] = u_vlocity[0];
	Deta[27] = u_vlocity[1];
	Deta[28] = u_vlocity[2];
	Deta[29] = u_vlocity[3];
	
  Deta[30] = v_vlocity[0];
	Deta[31] = v_vlocity[1];
	Deta[32] = v_vlocity[2];
	Deta[33] = v_vlocity[3];
	
	Deta[34] = w_vlocity[0];
	Deta[35] = w_vlocity[1];
	Deta[36] = w_vlocity[2];
	Deta[37] = w_vlocity[3];
	
	Deta[38] = p_vlocity[0];
	Deta[39] = p_vlocity[1];
	Deta[40] = p_vlocity[2];
	Deta[41] = p_vlocity[3];
	
	Deta[42] = q_vlocity[0];
	Deta[43] = q_vlocity[1];
	Deta[44] = q_vlocity[2];
	Deta[45] = q_vlocity[3];
	
	Deta[46] = r_vlocity[0];
	Deta[47] = r_vlocity[1];
	Deta[48] = r_vlocity[2];
	Deta[49] = r_vlocity[3];
	
	Deta[50] = 0xFF;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断回调函数
{
	static uint8_t rxd1_state = 0;//串口1的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd2_state = 0;//串口2的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd6_state = 0;//串口6的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd7_state = 0;//串口7的状态 --0：等待包头；1：接收数据；2：等待帧尾
	
	//DVL数据储存程序
	if(huart->Instance == USART1)
	{
		DVL_RxCallback(&dvl_parser, dvl_rx_byte);
    HAL_UART_Receive_IT(&huart1, &dvl_rx_byte, 1);                                                   
		
	 }			
				
	 
	//光纤惯导数据储存程序
	else if(huart->Instance == USART2)					
	{

		switch(rxd2_state)
					{
						case 0://等待帧头
							if(aRxBuffer2 == 0x80)
							{
								rxd2_state = 1;
								rxd2_index = 0;
							}
							else 
							{
								rxd2_state = 0;
							}
						  break;
						case 1://开始接收
							rxd2_buffer[rxd2_index] = aRxBuffer2; // 转存接收到的数据
						    rxd2_index++;
						  if(rxd2_index >= 9)//判断存满
							{
//								rxd2_state = 2;
								rxd2_flag = 1;//开始处理
								rxd2_state = 0;
							}
						  break;						
//						case 2:
//								rxd2_flag = 1;
//								rxd2_state = 0;
//						  break;						
					}
		 HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);	// 重新开启接收中断	
	}		
	
	
	
	//Xsense惯导数据储存程序
	else if(huart->Instance == USART6)					
	{
		 switch(rxd6_state)
					{
						case 0:
							if((aRxBuffer6 == 0xFA) && (rxd6_head == 0))
							{
								rxd6_head = 1;
							}
							else if((aRxBuffer6 == 0xFF) && (rxd6_head == 1))
							{
								rxd6_head = 2;
							}
							else if((aRxBuffer6 == 0x36) && (rxd6_head == 2))
							{
								rxd6_state = 1;
								rxd6_head = 0;
								rxd6_index = 0;
							}
							else 
							{
								rxd6_state = 0;
								rxd6_head = 0;
							}
						  break;
						case 1:
							rxd6_buffer[rxd6_index] = aRxBuffer6; // 转存接收到的数据
						    rxd6_index++;
						  if(rxd6_index >= 53)
							{
								rxd6_state = 2;
							}
						  break;						
						case 2:
								rxd6_flag = 1;
								rxd6_state = 0;
								rxd6_index = 0;
						  break;						
					}
		 HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer6, 1);	// 重新开启接收中断	
	}	
					
					
	//深度计数据储存程序
	else if(huart->Instance == UART7)					
	{
   
			  char ch = Depth_rx_buffer[0];
        
        if(ch == '\n')
        {
            Depth_rx_line[Depth_rx_index] = '\0';
            
            // 直接查找并解析D值
            char* p = strstr(Depth_rx_line, "D=");
            if(p) D_value = atof(p + 2);
            depth_num = get_D_value();
            Depth_rx_index = 0;
        }
        else if(ch != '\r' && Depth_rx_index < 49)
        {
            Depth_rx_line[Depth_rx_index++] = ch;
        }
		 HAL_UART_Receive_IT(&huart7, (uint8_t *)&Depth_rx_buffer, 1);	// 重新开启接收中断	
	}					
	
}		



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	 if (htim->Instance == htim2.Instance) // 确认是TIM2触发
    {
        if (huart2.gState == HAL_UART_STATE_READY)
        {
			HAL_UART_Transmit_IT(&huart2, (uint8_t *)"0", 1);
        }
    }	
	
	 if (htim->Instance == htim3.Instance) 
    {
        if (huart5.gState == HAL_UART_STATE_READY)
        {
			    Fuzhi();
			    HAL_UART_Transmit_IT(&huart5, Deta, 51);
					//Angle = Angle+0.0003;
					

        }
    }	
	
}


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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
	
  /* USER CODE BEGIN 2 */

	 
	 
	 DR_Init(&dr);
	 // 设置初始位置（可选）
   Position_t init_pos = {0.0f, 0.0f, 0.0f};
   // 配置优化参数
   IntegratorConfig_t config = {
        .use_trapezoidal = 1,        // 使用梯形积分
        .use_interpolation = 1,      // 使用插值
        .use_attitude_compensation = 1,
        .velocity_threshold = 0.005f, // 0.5cm/s噪声阈值
        .max_dt = 0.15f              // 最大150ms间隔
    };
	 DR_SetConfig(&dr, &config);
		
	 DVL_Init(&dvl_parser, &huart1);
	 HAL_UART_Receive_IT(&huart1, &dvl_rx_byte, 1);
	 
	 HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);
	 HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer6, 1);
	 HAL_UART_Receive_IT(&huart7, (uint8_t *)&Depth_rx_buffer, 1);
	
	 
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);   //光纤
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	

	 HAL_Delay(100);
	 HAL_TIM_Base_Start_IT(&htim2);//光纤惯导专用
	 
	 HAL_TIM_Base_Start_IT(&htim3);//光纤惯导专用
	
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		current_time = HAL_GetTick();
	  //USART6->XSense惯导、UART7->深度传感器、USART2->光纤惯导、USART1->UWB
		float tempu = 0.f;
		float tempv = 0.f;
		float tempw = 0.f;
		DVL_GetData(&dvl_parser, &speed_u, &speed_v, &speed_w);
		FloatToU8(speed_u/1000.f,u_vlocity);
		FloatToU8(speed_v/1000.f,v_vlocity);
		FloatToU8(speed_w/1000.f,w_vlocity);
		
	  
	  if(rxd2_flag == 1)//进入光纤惯导解算程序
	  {
			Angleadd = Angle_function(rxd2_buffer);
			Angle -= Angleadd;
			FloatToU8(Angle,yaw);
		 
			b++; 
			attitude.yaw = Angle*M_PI/180;
			DR_UpdateAttitude(&dr, &attitude, current_time);
			rxd2_flag = 0;  
	  }
	  
		if(rxd6_flag == 1)//进入XSense惯导解算程序
	  {
			Pitch = Yaw_function(rxd6_buffer);
			roll = Pitch_function(rxd6_buffer);			
			speed_p = p_function(rxd6_buffer);
			speed_q = q_function(rxd6_buffer);
			speed_r = r_function(rxd6_buffer);
			
			speed_p = -1*speed_p;
			speed_q = -1*speed_q;
			speed_r = -1*speed_r;
			roll = -1*roll;
			Pitch = -1*Pitch;	
			
			//p q交换以适配载体坐标系以及惯导盒布置
			FloatToU8(speed_q,p_vlocity);
			FloatToU8(speed_p,q_vlocity);
			FloatToU8(speed_r,r_vlocity);
			FloatToU8(Pitch,pitch);
			FloatToU8(roll,Roll);
      
			attitude.roll = roll*M_PI/180.f;     
      attitude.pitch = Pitch*M_PI/180.f;  

			rxd6_flag = 0; 
	  }
	  
		FloatToU8(depth_num,Depth);
	
		if(dvl_parser.updated)
		{
			if((speed_u <2000.f)&&(speed_v<2000.f)&&speed_w<2000.f)
	    {
	    	body_velocity.u = speed_u*0.001f;
	    	body_velocity.v = speed_v*0.001f;
	    	body_velocity.w = speed_w*0.001f;
				DR_UpdateVelocity(&dr, &body_velocity, current_time);
	    }
		}
		
		DR_UpdateDepth(&dr, depth_num);
		
		//航位推算
		if (current_time - last_position_update >= 20) { // 50Hz
            DR_UpdatePosition(&dr, current_time);
            last_position_update = current_time;

            DR_GetPosition(&dr, &pos);
						FloatToU8(pos.x,Xpos);
						FloatToU8(pos.y,Ypos);
            
        }
		
		
		

		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
