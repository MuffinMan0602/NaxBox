#ifndef _DEPTH_SENSOR_H_
#define _DEPTH_SENSOR_H_

#include "main.h"

#define rxd7_size 30  //��ȼƹߵ����ݻ���������
extern uint8_t aRxBuffer7;//���ݽ����ݴ�
extern uint8_t rxd7_buffer[rxd7_size];//�������ݻ�������
extern uint8_t rxd7_flag;//���ݴ洢��־λ
extern uint8_t rxd7_index;//�����ֽ�����
extern uint8_t depth[4];

float Depth_function(uint8_t rxd7_buffer[rxd7_size]);


#endif