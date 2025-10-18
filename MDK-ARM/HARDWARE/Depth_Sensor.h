#ifndef _DEPTH_SENSOR_H_
#define _DEPTH_SENSOR_H_

#include "main.h"

#define rxd7_size 30  //深度计惯导数据缓冲区长度
extern uint8_t aRxBuffer7;//数据接收暂存
extern uint8_t rxd7_buffer[rxd7_size];//定义数据缓冲数组
extern uint8_t rxd7_flag;//数据存储标志位
extern uint8_t rxd7_index;//接收字节索引
extern uint8_t depth[4];

float Depth_function(uint8_t rxd7_buffer[rxd7_size]);


#endif