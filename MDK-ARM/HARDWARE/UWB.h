#ifndef _UWB_H_
#define _UWB_H_

#include "main.h"
#include "U8_Float.h"

#define rxd1_size 172  //UWB���ݻ���������
extern uint8_t aRxBuffer1;//���ݽ����ݴ�
extern uint8_t rxd1_buffer[rxd1_size];//�������ݻ�������
extern uint8_t rxd1_flag;//���ݴ洢��־λ
extern uint8_t rxd1_index;//�����ֽ�����
extern uint8_t rxd1_head;//�жϱ�־λ
extern uint8_t XaxisBuffer[3];
extern uint8_t YaxisBuffer[3];


extern UART_HandleTypeDef huart1;

float X_function(uint8_t rxd1_buffer[rxd1_size]);

float Y_function(uint8_t rxd1_buffer[rxd1_size]);

#endif