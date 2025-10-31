/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author			: ��ԣ��
  * @brief			: ������������H743\H753�����壬�������˹ߵ���XSense�ߵ���UWBλ�ô���������ȴ�����
  * @introduction	: USART6->XSense�ߵ���  UART7->��ȴ�������  UART5->��RCT6ͨ��74800��  USART2->���˹ߵ���  USART1->UWB
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
#include <stdbool.h>   // ��Ӳ�������֧��
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ȫ�ֱ���

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/*��Զ����Ҫ�޸����ݣ��޸ĵ�������ʽΪ��

//ע������
�޸ı�����
//ע������

1.�Ϲ���⵱ǰʱ������һ�ν���ʱ��Ĳ���Ϊ����uwb�źŵ����ݣ�

     ͨ��ʱ����ж�UWB�����Ƿ�ʱ

     ���ó�ʱʱ��Ϊ200ms

2.��UWB�ж��и���ʱ�����

     ���յ�UWB����ʱ������last_uwb_time

3.�޸�λ�ý����߼���

     ��uwb_valid_flagΪ�棬ʹ��UWB�����XYλ��

     ��uwb_valid_flagΪ�٣�ʹ��DVL�����XYλ��

4.������֡�����UWB״̬��

     ��uwb_valid_flagֵ��ӵ�Deta[2]λ�ã�����ָʾUWB�ź�״̬

5.����ѭ���ж��ڼ��UWB�źţ�

     ÿ��ѭ�������ü��ʱ���������UWB״̬
*/



//���ӵ�ָʾλ
bool uwb_valid_flag = true; // UWB�ź�״ָ̬ʾ����uwbΪ�٣�ʹ��dvl��λ����λ�ã���uwbΪ�棬ʹ��uwb�źż���λ��
//���ӵ�ָʾλ


//����Ϊʵ�������������
DVL_Parser_t dvl_parser;
uint8_t dvl_rx_byte;
DeadReckoning_t dr;
uint32_t last_position_update = 0;
Position_t pos;

Velocity_t body_velocity;
Attitude_t attitude;      
uint32_t current_time;
Position_t current_pos;

float sensor_u, sensor_v, sensor_w;           // �ٶ�����
float sensor_roll, sensor_pitch, sensor_yaw;  // ��̬����
float sensor_depth;                           // �������

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


// �ж�����UWB��ر�������
uint32_t last_uwb_time = 0;
#define UWB_TIMEOUT 200  // UWB��ʱʱ��200ms
// �ж�����UWB��ر�������


//���巢�͵�����
uint8_t Xpos[4] = {0};//X
uint8_t Ypos[4] = {0};
uint8_t Depth[4] = {0};

uint8_t Roll[4] = {0};
uint8_t pitch[4] = {0};//������float�����ˣ���Сд
uint8_t yaw[4] = {0};

uint8_t u_vlocity[4] = {0};
uint8_t v_vlocity[4] = {0};
uint8_t w_vlocity[4] = {0};

uint8_t p_vlocity[4] = {0};
uint8_t q_vlocity[4] = {0};
uint8_t r_vlocity[4] = {0};


//��51�����52������ָʾλzsw
uint8_t Deta[52] = {0};//��������
//��51�����52������ָʾλzsw





uint8_t UART6_Rec_Buf[50] = {0};
uint8_t UART7_Rec_Buf[50] = {0};

uint8_t UART6_Rec_Counter = 0;
uint8_t UART7_Rec_Counter = 0;
//��ȼ�ȫ�ֱ���
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
	
	//������uwbָʾλ��
	Deta[2] = uwb_valid_flag;
	//������uwbָʾλ��
	
	
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//�����жϻص�����
{
	static uint8_t rxd1_state = 0;//����1��״̬ --0���ȴ���ͷ��1���������ݣ�2���ȴ�֡β
	static uint8_t rxd2_state = 0;//����2��״̬ --0���ȴ���ͷ��1���������ݣ�2���ȴ�֡β
	static uint8_t rxd6_state = 0;//����6��״̬ --0���ȴ���ͷ��1���������ݣ�2���ȴ�֡β
	static uint8_t rxd7_state = 0;//����7��״̬ --0���ȴ���ͷ��1���������ݣ�2���ȴ�֡β
	
	
	//������uwb����
	static uint8_t rxd3_state = 0;//����3��״̬ --0���ȴ���ͷ��1���������ݣ�2���ȴ�֡β
	//������uwb����
	
	
	//DVL���ݴ������
	if(huart->Instance == USART1)
	{
		DVL_RxCallback(&dvl_parser, dvl_rx_byte);
    HAL_UART_Receive_IT(&huart1, &dvl_rx_byte, 1);                                                   
		
	 }			
				
	 
	//���˹ߵ����ݴ������
	else if(huart->Instance == USART2)					
	{

		switch(rxd2_state)
					{
						case 0://�ȴ�֡ͷ
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
						case 1://��ʼ����
							rxd2_buffer[rxd2_index] = aRxBuffer2; // ת����յ�������
						    rxd2_index++;
						  if(rxd2_index >= 9)//�жϴ���
							{
//								rxd2_state = 2;
								rxd2_flag = 1;//��ʼ����
								rxd2_state = 0;
							}
						  break;						
//						case 2:
//								rxd2_flag = 1;
//								rxd2_state = 0;
//						  break;						
					}
		 HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);	// ���¿��������ж�	
	}		
	
	
	
	//Xsense�ߵ����ݴ������
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
							rxd6_buffer[rxd6_index] = aRxBuffer6; // ת����յ�������
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
		 HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer6, 1);	// ���¿��������ж�	
	}	
					
					
	//��ȼ����ݴ������
	else if(huart->Instance == UART7)					
	{
   
			  char ch = Depth_rx_buffer[0];
        
        if(ch == '\n')
        {
            Depth_rx_line[Depth_rx_index] = '\0';
            
            // ֱ�Ӳ��Ҳ�����Dֵ
            char* p = strstr(Depth_rx_line, "D=");
            if(p) D_value = atof(p + 2);
            depth_num = get_D_value();
            Depth_rx_index = 0;
        }
        else if(ch != '\r' && Depth_rx_index < 49)
        {
            Depth_rx_line[Depth_rx_index++] = ch;
        }
		 HAL_UART_Receive_IT(&huart7, (uint8_t *)&Depth_rx_buffer, 1);	// ���¿��������ж�	
	}

	
	
	//������UWB���ݴ������
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
								
								// �յ�UWB���ݣ�����ʱ���
								last_uwb_time = HAL_GetTick();
								// �յ�UWB���ݣ�����ʱ���
								
								
							}
							else 
							{
								rxd3_state = 0;
								rxd3_head = 0;
							}
						  break;						
							
						case 1:
							rxd3_buffer[rxd3_index] = aRxBuffer3; // ת����յ�������
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
		 HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);	// ���¿��������ж�	
	 }			
	//������UWB���ݴ������
	 
	 
	
}		



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	 if (htim->Instance == htim2.Instance) // ȷ����TIM2����
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
	 // ���ó�ʼλ�ã���ѡ��
   Position_t init_pos = {0.0f, 0.0f, 0.0f};
   // �����Ż�����
   IntegratorConfig_t config = {
        .use_trapezoidal = 1,        // ʹ�����λ���
        .use_interpolation = 1,      // ʹ�ò�ֵ
        .use_attitude_compensation = 1,
        .velocity_threshold = 0.005f, // 0.5cm/s������ֵ
        .max_dt = 0.15f              // ���150ms���
    };
	 DR_SetConfig(&dr, &config);
		
	 DVL_Init(&dvl_parser, &huart1);
	
//ԭ�����жϣ���û�п���DMA,	
//	 HAL_UART_Receive_IT(&huart1, &dvl_rx_byte, 1);
//	 
//	 HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);
//	
//	//���������Ĵ��ڽ���
//	 HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);
//	//���������Ĵ��ڽ���
//	
//	 HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer6, 1);
//	 HAL_UART_Receive_IT(&huart7, (uint8_t *)&Depth_rx_buffer, 1);
	
	
//DMA����
	 HAL_UART_Receive_DMA(&huart1, &dvl_rx_byte, 1);
	 
	  HAL_UART_Receive_DMA(&huart2, (uint8_t *)&aRxBuffer2, 1);
	
	//���������Ĵ��ڽ���
	  HAL_UART_Receive_DMA(&huart3, (uint8_t *)&aRxBuffer3, 1);
	//���������Ĵ��ڽ���
	
	  HAL_UART_Receive_DMA(&huart6, (uint8_t *)&aRxBuffer6, 1);
	 HAL_UART_Receive_DMA(&huart7, (uint8_t *)&Depth_rx_buffer, 1);	

	 
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);   //����
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	

	 HAL_Delay(100);
	 HAL_TIM_Base_Start_IT(&htim2);//���˹ߵ�ר��
	 
	 HAL_TIM_Base_Start_IT(&htim3);//���˹ߵ�ר��
	
   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  current_time = HAL_GetTick();
	  
if((current_time - last_uwb_time) > UWB_TIMEOUT)
    {
        uwb_valid_flag = false; // ��ǰʱ������һ��uwb�����ź�ʱ�䳬��200ms���ж�Ϊuwb�źŶϿ�
    }
    else
    {
		uwb_valid_flag = true;  // �ж�Ϊuwb�ź���Ч
    }
	  
		
	  
	  //USART6->XSense�ߵ���UART7->��ȴ�������USART2->���˹ߵ���USART1->UWB
		float tempu = 0.f;
		float tempv = 0.f;
		float tempw = 0.f;
		DVL_GetData(&dvl_parser, &speed_u, &speed_v, &speed_w);
		FloatToU8(speed_u/1000.f,u_vlocity);
		FloatToU8(speed_v/1000.f,v_vlocity);
		FloatToU8(speed_w/1000.f,w_vlocity);
		
	  
	  
//�����ģ�����UWB�������
	   if(rxd3_flag == 1)
	  {
		Xaxis=X_function(rxd3_buffer);
		Yaxis=Y_function(rxd3_buffer);
		  
        FloatToU8(Xaxis, Xpos);
        FloatToU8(Yaxis, Ypos);
        
        // ����DRλ�õ���ǰUWBλ��
        DR_Reset(&dr, Xaxis, Yaxis, depth_num);
   
		rxd3_flag = 0; 
	  }
//�����ģ�����UWB�������
	      
	  

	  
	  
	  if(rxd2_flag == 1)//������˹ߵ��������
	  {
			Angleadd = Angle_function(rxd2_buffer);
			Angle -= Angleadd;
			FloatToU8(Angle,yaw);
		  
			attitude.yaw = Angle*M_PI/180;
			DR_UpdateAttitude(&dr, &attitude, current_time);
			rxd2_flag = 0;  
	  }
	  
		if(rxd6_flag == 1)//����XSense�ߵ��������
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
			
			//p q������������������ϵ�Լ��ߵ��в���
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
		
			//��λ���� - ֻ�е�UWB�ź���Чʱ��ʹ��DVL�����λ��
		if (current_time - last_position_update >= 20) 
			
		{ // 50Hz
			if(!uwb_valid_flag)  // ʹ�ò���ֵ�ж�
			{
            DR_UpdatePosition(&dr, current_time);
            last_position_update = current_time;
				
            DR_GetPosition(&dr, &pos);
				
            FloatToU8(pos.x,Xpos);
            FloatToU8(pos.y,Ypos);
            }
        }
		//��λ���� - ֻ�е�UWB�ź���Чʱ��ʹ��DVL�����λ��
		
		

		
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
