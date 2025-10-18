#ifndef _GUANGXIAN_H_
#define _GUANGXIAN_H_

#include "main.h"

#define rxd2_size 10  //光纤惯导数据缓冲区长度
extern uint8_t aRxBuffer2;//数据接收暂存
extern uint8_t rxd2_buffer[rxd2_size];//定义数据缓冲数组
extern uint8_t rxd2_flag;//数据存储标志位
extern uint8_t rxd2_index;//接收字节索引
extern uint8_t AngleBuffer[5];
extern float Angleadd;
extern int Angleresult;

float Angle_function(uint8_t rxd2_buffer[rxd2_size]);


#endif