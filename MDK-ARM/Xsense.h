#ifndef _XSENSE_H_
#define _XSENSE_H_

#include "main.h"
#include "U8_Float.h"

#define rxd6_size 57  //Xsense�ߵ����ݻ���������
extern uint8_t aRxBuffer6;//���ݽ����ݴ�
extern uint8_t rxd6_buffer[rxd6_size];//�������ݻ�������
extern uint8_t rxd6_flag;//���ݴ洢��־λ
extern uint8_t rxd6_index;//�����ֽ�����
extern uint8_t rxd6_head;
extern float Yaw_last;
extern float Pitch_last;
extern uint8_t PitchBuffer[4];
extern uint8_t YawBuffer[4];

float Pitch_function(uint8_t rxd6_buffer[rxd6_size]);

float Yaw_function(uint8_t rxd6_buffer[rxd6_size]);

float p_function(uint8_t rxd6_buffer[rxd6_size]);
float q_function(uint8_t rxd6_buffer[rxd6_size]);
float r_function(uint8_t rxd6_buffer[rxd6_size]);

#endif