#ifndef _XSENSE_H_
#define _XSENSE_H_

#include "main.h"
#include "U8_Float.h"

#define rxd6_size 57  //Xsense惯导数据缓冲区长度
extern uint8_t aRxBuffer6;//数据接收暂存
extern uint8_t rxd6_buffer[rxd6_size];//定义数据缓冲数组
extern uint8_t rxd6_flag;//数据存储标志位
extern uint8_t rxd6_index;//接收字节索引
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