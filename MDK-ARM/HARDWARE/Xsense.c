#include "Xsense.h"
#include "main.h"
#include "U8_Float.h"



#define rxd6_size 57  //Xsense惯导数据缓冲区长度
uint8_t aRxBuffer6;//数据接收暂存
uint8_t rxd6_buffer[rxd6_size];//定义数据缓冲数组
uint8_t rxd6_flag = 0;//数据存储标志位
uint8_t rxd6_index = 0;//接收字节索引
uint8_t rxd6_head = 0;
float Yaw_last=0;
float Pitch_last=0;
float p_last = 0;
float q_last = 0;
float r_last = 0;

uint8_t PitchBuffer[4]={0};
uint8_t YawBuffer[4]={0};
uint8_t pBuffer[4] = {0};
uint8_t qBuffer[4] = {0};
uint8_t rBuffer[4] = {0};


float Pitch_function(uint8_t rxd6_buffer[rxd6_size])
{		
		float a=0;

		PitchBuffer[0]=rxd6_buffer[8];
		PitchBuffer[1]=rxd6_buffer[9];
		PitchBuffer[2]=rxd6_buffer[10];
		PitchBuffer[3]=rxd6_buffer[11];
	
		a=U8ToFloat(PitchBuffer);
	
		if(fabs (a-Pitch_last)>90)
					 {
					  a = Pitch_last;
					 }
				 
		Pitch_last = a;
					 
		return a;

}

float Yaw_function(uint8_t rxd6_buffer[rxd6_size])
{		
		float b=0;

	
		YawBuffer[0]=rxd6_buffer[4];
		YawBuffer[1]=rxd6_buffer[5];
		YawBuffer[2]=rxd6_buffer[6];
		YawBuffer[3]=rxd6_buffer[7];
	
//		YawBuffer[0]=rxd6_buffer[12];
//		YawBuffer[1]=rxd6_buffer[13];
//		YawBuffer[2]=rxd6_buffer[14];
//		YawBuffer[3]=rxd6_buffer[15];
	
		b=U8ToFloat(YawBuffer);
	
		if(fabs (b-Yaw_last)>180)
					 {
					  b = Yaw_last;
					 }
				 
		Yaw_last = b;
					 
		return b;

}

float p_function(uint8_t rxd6_buffer[rxd6_size])
{		
		float b=0;

		pBuffer[0]=rxd6_buffer[19];
		pBuffer[1]=rxd6_buffer[20];
		pBuffer[2]=rxd6_buffer[21];
		pBuffer[3]=rxd6_buffer[22];
	
		b=U8ToFloat(pBuffer);
	
		if(fabs (b-p_last)>180)
					 {
					  b = p_last;
					 }
				 
		p_last = b;
					 
		return b;
}


float q_function(uint8_t rxd6_buffer[rxd6_size])
{		
		float b=0;

		qBuffer[0]=rxd6_buffer[23];
		qBuffer[1]=rxd6_buffer[24];
		qBuffer[2]=rxd6_buffer[25];
		qBuffer[3]=rxd6_buffer[26];
	
		b=U8ToFloat(qBuffer);
	
		if(fabs (b-q_last)>180)
					 {
					  b = q_last;
					 }
				 
		q_last = b;
					 
		return b;
}

float r_function(uint8_t rxd6_buffer[rxd6_size])
{		
		float b=0;

		rBuffer[0]=rxd6_buffer[27];
		rBuffer[1]=rxd6_buffer[28];
		rBuffer[2]=rxd6_buffer[29];
		rBuffer[3]=rxd6_buffer[30];
	
		b=U8ToFloat(rBuffer);
	
		if(fabs (b-r_last)>180)
					 {
					  b = r_last;
					 }
				 
		r_last = b;
					 
		return b;
}