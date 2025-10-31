#ifndef _UWB_H_
#define _UWB_H_

#include "main.h"
#include "U8_Float.h"

#define rxd3_size 124  //UWB���ݻ���������
extern uint8_t aRxBuffer3;//���ݽ����ݴ�
extern uint8_t rxd3_buffer[rxd3_size];//�������ݻ�������
extern uint8_t rxd3_flag;//���ݴ洢��־λ
extern uint8_t rxd3_index;//�����ֽ�����
extern uint8_t rxd3_head;//�жϱ�־λ
extern uint8_t XaxisBuffer[3];
extern uint8_t YaxisBuffer[3];


extern UART_HandleTypeDef huart3;

float X_function(uint8_t rxd3_buffer[rxd3_size]);

float Y_function(uint8_t rxd3_buffer[rxd3_size]);

#endif