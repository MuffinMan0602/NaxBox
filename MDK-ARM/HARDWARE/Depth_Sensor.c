#include "Depth_Sensor.h"
#include "main.h"


#define rxd7_size 30  //Xsense�ߵ����ݻ���������
uint8_t aRxBuffer7;//���ݽ����ݴ�
uint8_t rxd7_buffer[rxd7_size];//�������ݻ�������
uint8_t rxd7_flag = 0;//���ݴ洢��־λ
uint8_t rxd7_index = 0;//�����ֽ�����
uint8_t depth[4];

float Depth_function(uint8_t rxd7_buffer[rxd7_size])
{		
			float a = 0;
	    if (rxd7_buffer[5] == 0x2D)
			{
			depth[0] = rxd7_buffer[6];
			depth[1] = rxd7_buffer[7];
			depth[2] = rxd7_buffer[8];
			depth[3] = rxd7_buffer[9];
			a = 0 - (depth[0]-48) - (depth[2]-48)*0.1 - (depth[3]-48)*0.01;
			}
			else
			{
			depth[0] = rxd7_buffer[5];
			depth[1] = rxd7_buffer[6];
			depth[2] = rxd7_buffer[7];
			depth[3] = rxd7_buffer[8];
			a =(depth[0]-48) + (depth[2]-48)*0.1 + (depth[3]-48)*0.01;
			}

		return a;
}