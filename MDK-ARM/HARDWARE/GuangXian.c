#include "GuangXian.h"
#include "main.h"

#define rxd2_size 10  //光纤惯导数据缓冲区长度
uint8_t aRxBuffer2=0;//数据接收暂存
uint8_t rxd2_buffer[rxd2_size]={0};//定义数据缓冲数组
uint8_t rxd2_flag = 0;//数据存储标志位
uint8_t rxd2_index = 0;//接收字节索引

float Angleadd = 0.0;
int Angleresult = 0;

uint8_t AngleBuffer[5]={0};

extern UART_HandleTypeDef huart2;

float Angle_function(uint8_t rxd2_buffer[rxd2_size])
{
							Angleadd = 0;
	
                            AngleBuffer[0] = rxd2_buffer[4];
							AngleBuffer[1] = rxd2_buffer[3];
							AngleBuffer[2] = rxd2_buffer[2];
							AngleBuffer[3] = rxd2_buffer[1];
							AngleBuffer[4] = rxd2_buffer[0];

							// 清零 result
							Angleresult = 0;
	
							// 提取 AngleBuffer 的数据
	
							Angleresult |= (AngleBuffer[0] & 0x7F);
							Angleresult <<= 7;
							Angleresult |= (AngleBuffer[1] & 0x7F);
							Angleresult <<= 7;
							Angleresult |= (AngleBuffer[2] & 0x7F);
							Angleresult <<= 7;
							Angleresult |= (AngleBuffer[3] & 0x7F);
							Angleresult <<= 7;
							Angleresult |= (AngleBuffer[4] & 0x7F);	
							
							Angleadd = Angleresult/132805253.0;

                            return Angleadd;	
}


