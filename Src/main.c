/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author			: 宋裕恒
  * @brief			: 本程序适用于H743\H753开发板，包括光纤惯导、XSense惯导、UWB位置传感器、深度传感器
  * @introduction	: USART6->XSense惯导、  UART7->深度传感器、  UART5->与RCT6通信74800、  USART2->光纤惯导、  USART1->UWB
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
#include "memorymap.h"
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
#include <stdbool.h>   // 添加布尔类型支持
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



/*黄远标主要修改内容：修改的内容形式为：

//注释内容
修改本部分
//注释内容

1.拖过检测当前时间与上一次接收时间的差作为有无uwb信号的依据：

     通过时间戳判断UWB数据是否超时

     设置超时时间为200ms

2.在UWB中断中更新时间戳：

     当收到UWB数据时，更新last_uwb_time

3.修改位置解算逻辑：

     当uwb_valid_flag为真，使用UWB解算的XY位置

     当uwb_valid_flag为假，使用DVL解算的XY位置

4.在数据帧中添加UWB状态：

     将uwb_valid_flag值添加到Deta[2]位置，用于指示UWB信号状态

5.在主循环中定期检查UWB信号：

     每次循环都调用检测时间差来更新UWB状态
*/



//增加的指示位
bool uwb_valid_flag = true; // UWB信号状态指示：无uwb为假，使用dvl推位计算位置，有uwb为真，使用uwb信号计算位置
//增加的指示位


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


// 判断有无UWB相关变量依据
uint32_t last_uwb_time = 0;
#define UWB_TIMEOUT 200  // UWB超时时间200ms
// 判断有无UWB相关变量依据


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


//从51变成了52，多了指示位zsw
uint8_t Deta[52] = {0};//最后的数据
//从51变成了52，多了指示位zsw





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
	
	//新增的uwb指示位置
	Deta[2] = uwb_valid_flag;
	//新增的uwb指示位置
	
	
	Deta[3] = Xpos[0];
	Deta[4] = Xpos[1];
	Deta[5] = Xpos[2];
	Deta[6] = Xpos[3];
	
	Deta[7] = Ypos[0];
	Deta[8] = Ypos[1];
	Deta[9] = Ypos[2];
	Deta[10] = Ypos[3];
	
	Deta[11] = Depth[0];
	Deta[12] = Depth[1];
	Deta[13] = Depth[2];
	Deta[14] = Depth[3];
	
	Deta[15] = Roll[0];
	Deta[16] = Roll[1];
	Deta[17] = Roll[2];
	Deta[18] = Roll[3];
	
	Deta[19] = pitch[0];
	Deta[20] = pitch[1];
	Deta[21] = pitch[2];
	Deta[22] = pitch[3];
	
	Deta[23] = yaw[0];
	Deta[24] = yaw[1];
	Deta[25] = yaw[2];
	Deta[26] = yaw[3];
	
	Deta[27] = u_vlocity[0];
	Deta[28] = u_vlocity[1];
	Deta[29] = u_vlocity[2];
	Deta[30] = u_vlocity[3];
	
	Deta[31] = v_vlocity[0];
	Deta[32] = v_vlocity[1];
	Deta[33] = v_vlocity[2];
	Deta[34] = v_vlocity[3];
	
	Deta[35] = w_vlocity[0];
	Deta[36] = w_vlocity[1];
	Deta[37] = w_vlocity[2];
	Deta[38] = w_vlocity[3];
	
	Deta[39] = p_vlocity[0];
	Deta[40] = p_vlocity[1];
	Deta[41] = p_vlocity[2];
	Deta[42] = p_vlocity[3];
	
	Deta[43] = q_vlocity[0];
	Deta[44] = q_vlocity[1];
	Deta[45] = q_vlocity[2];
	Deta[46] = q_vlocity[3];
	
	Deta[47] = r_vlocity[0];
	Deta[48] = r_vlocity[1];
	Deta[49] = r_vlocity[2];
	Deta[50] = r_vlocity[3];
	
	Deta[51] = 0xFF;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口中断回调函数
{
	static uint8_t rxd1_state = 0;//串口1的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd2_state = 0;//串口2的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd6_state = 0;//串口6的状态 --0：等待包头；1：接收数据；2：等待帧尾
	static uint8_t rxd7_state = 0;//串口7的状态 --0：等待包头；1：接收数据；2：等待帧尾
	
	
	//新增的uwb串口
	static uint8_t rxd3_state = 0;//串口3的状态 --0：等待包头；1：接收数据；2：等待帧尾
	//新增的uwb串口
	
	
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

	
	
	//新增的UWB数据储存程序
	if(huart->Instance == USART3)
	{
		switch(rxd3_state)
					{
						case 0:
							if((aRxBuffer3 == 0x55) && (rxd3_head == 0))
							{
								rxd3_head = 1;
							}
							else if((aRxBuffer3 == 0x01) && (rxd3_head == 1))
							{
								rxd3_head = 2;
							}
							else if((aRxBuffer3 == 0x00) && (rxd3_head == 2))
							{
								rxd3_state = 1;
								rxd3_head = 0;
								rxd3_index = 0;
								
								// 收到UWB数据，更新时间戳
								last_uwb_time = HAL_GetTick();
								// 收到UWB数据，更新时间戳
								
								
							}
							else 
							{
								rxd3_state = 0;
								rxd3_head = 0;
							}
						  break;						
							
						case 1:
							rxd3_buffer[rxd3_index] = aRxBuffer3; // 转存接收到的数据
							rxd3_index++;
						  if(rxd3_index >= 124)
							{
								rxd3_state = 2;
							}
						  break;
							
						case 2:
								rxd3_flag = 1;
								rxd3_state = 0;
								rxd3_index = 0;
						  break;						
					}
		 HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);	// 重新开启接收中断	
	 }			
	//新增的UWB数据储存程序
	 
	 
	
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
			    HAL_UART_Transmit_IT(&huart5, Deta, 52);
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
  MX_USART3_UART_Init();
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
	
//原来是中断，并没有开启DMA,	
//	 HAL_UART_Receive_IT(&huart1, &dvl_rx_byte, 1);
//	 
//	 HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);
//	
//	//开启新增的串口接收
//	 HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);
//	//开启新增的串口接收
//	
//	 HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer6, 1);
//	 HAL_UART_Receive_IT(&huart7, (uint8_t *)&Depth_rx_buffer, 1);
	
	
//DMA开启
	 HAL_UART_Receive_DMA(&huart1, &dvl_rx_byte, 1);
	 
	  HAL_UART_Receive_DMA(&huart2, (uint8_t *)&aRxBuffer2, 1);
	
	//开启新增的串口接收
	  HAL_UART_Receive_DMA(&huart3, (uint8_t *)&aRxBuffer3, 1);
	//开启新增的串口接收
	
	  HAL_UART_Receive_DMA(&huart6, (uint8_t *)&aRxBuffer6, 1);
	 HAL_UART_Receive_DMA(&huart7, (uint8_t *)&Depth_rx_buffer, 1);	

	 
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
	  
if((current_time - last_uwb_time) > UWB_TIMEOUT)
    {
        uwb_valid_flag = false; // 当前时间与上一次uwb接收信号时间超过200ms，判断为uwb信号断开
    }
    else
    {
		uwb_valid_flag = true;  // 判断为uwb信号有效
    }
	  
		
	  
	  //USART6->XSense惯导、UART7->深度传感器、USART2->光纤惯导、USART1->UWB
		float tempu = 0.f;
		float tempv = 0.f;
		float tempw = 0.f;
		DVL_GetData(&dvl_parser, &speed_u, &speed_v, &speed_w);
		FloatToU8(speed_u/1000.f,u_vlocity);
		FloatToU8(speed_v/1000.f,v_vlocity);
		FloatToU8(speed_w/1000.f,w_vlocity);
		
	  
	  
//新增的：进入UWB解算程序
	   if(rxd3_flag == 1)
	  {
		Xaxis=X_function(rxd3_buffer);
		Yaxis=Y_function(rxd3_buffer);
		  
        FloatToU8(Xaxis, Xpos);
        FloatToU8(Yaxis, Ypos);
        
        // 重置DR位置到当前UWB位置
        DR_Reset(&dr, Xaxis, Yaxis, depth_num);
   
		rxd3_flag = 0; 
	  }
//新增的：进入UWB解算程序
	      
	  

	  
	  
	  if(rxd2_flag == 1)//进入光纤惯导解算程序
	  {
			Angleadd = Angle_function(rxd2_buffer);
			Angle -= Angleadd;
			FloatToU8(Angle,yaw);
		  
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
		
			//航位推算 - 只有当UWB信号无效时才使用DVL解算的位置
		if (current_time - last_position_update >= 20) 
			
		{ // 50Hz
			if(!uwb_valid_flag)  // 使用布尔值判断
			{
            DR_UpdatePosition(&dr, current_time);
            last_position_update = current_time;
				
            DR_GetPosition(&dr, &pos);
				
            FloatToU8(pos.x,Xpos);
            FloatToU8(pos.y,Ypos);
            }
        }
		//航位推算 - 只有当UWB信号无效时才使用DVL解算的位置
		
		

		
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
