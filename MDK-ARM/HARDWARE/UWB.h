#ifndef _UWB_H_
#define _UWB_H_

#include "main.h"
#include "U8_Float.h"

#define rxd3_size 124  //UWB数据缓冲区长度
extern uint8_t aRxBuffer3;//数据接收暂存
extern uint8_t rxd3_buffer[rxd3_size];//定义数据缓冲数组
extern uint8_t rxd3_flag;//数据存储标志位
extern uint8_t rxd3_index;//接收字节索引
extern uint8_t rxd3_head;//判断标志位
extern uint8_t XaxisBuffer[3];
extern uint8_t YaxisBuffer[3];


extern UART_HandleTypeDef huart3;

float X_function(uint8_t rxd3_buffer[rxd3_size]);

float Y_function(uint8_t rxd3_buffer[rxd3_size]);

#endif