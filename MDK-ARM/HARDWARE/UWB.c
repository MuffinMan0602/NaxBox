#include "UWB.h"
#include "main.h"
#include "U8_Float.h"

#define rxd3_size 124  //UWB数据缓冲区长度
uint8_t aRxBuffer3;//数据接收暂存
uint8_t rxd3_buffer[rxd3_size];//定义数据缓冲数组
uint8_t rxd3_flag = 0;//数据存储标志位
uint8_t rxd3_index = 0;//接收字节索引
uint8_t rxd3_head = 0;//判断标志位
uint8_t XaxisBuffer[3] = {0};
uint8_t YaxisBuffer[3] = {0};

extern UART_HandleTypeDef huart3;

float X_function(uint8_t rxd3_buffer[rxd3_size])
{		float  X=0;
		XaxisBuffer[0] = rxd3_buffer[10];
		XaxisBuffer[1] = rxd3_buffer[11];
		XaxisBuffer[2] = rxd3_buffer[12];
		X = (int32_t)((XaxisBuffer[0] << 8)  |
						  (XaxisBuffer[1] << 16) |
						  (XaxisBuffer[2] << 24)) / 256.0f / 1000.0f;
		return X;
}
		
float Y_function(uint8_t rxd3_buffer[rxd3_size])
{		float  Y=0;
		YaxisBuffer[0] = rxd3_buffer[13];
		YaxisBuffer[1] = rxd3_buffer[14];
		YaxisBuffer[2] = rxd3_buffer[15];
		Y = (int32_t)((YaxisBuffer[0] << 8)  |
						  (YaxisBuffer[1] << 16) |
						  (YaxisBuffer[2] << 24)) / 256.0f / 1000.0f;
		return Y;
}	 
	 