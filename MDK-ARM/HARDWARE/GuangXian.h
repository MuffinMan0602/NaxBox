#ifndef _GUANGXIAN_H_
#define _GUANGXIAN_H_

#include "main.h"

#define rxd2_size 10  //���˹ߵ����ݻ���������
extern uint8_t aRxBuffer2;//���ݽ����ݴ�
extern uint8_t rxd2_buffer[rxd2_size];//�������ݻ�������
extern uint8_t rxd2_flag;//���ݴ洢��־λ
extern uint8_t rxd2_index;//�����ֽ�����
extern uint8_t AngleBuffer[5];
extern float Angleadd;
extern int Angleresult;

float Angle_function(uint8_t rxd2_buffer[rxd2_size]);


#endif