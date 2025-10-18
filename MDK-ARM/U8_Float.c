#include "U8_Float.h"
#include "main.h"

uint8_t U8Send[4] = {0};
uint8_t U8Data[4] = {0};
float FloatSend = 0;


float U8ToFloat(uint8_t U8Data[4])
{
	float FloatData=0;
	*((char *)(&FloatData))     = U8Data[3];
	*((char *)(&FloatData) + 1) = U8Data[2];
	*((char *)(&FloatData) + 2) = U8Data[1];
	*((char *)(&FloatData) + 3) = U8Data[0];
	return FloatData;
}

void FloatToU8(float FloatSend ,uint8_t U8Send[4])
{
		U8Send[3] = *((char *)(&FloatSend));
		U8Send[2] = *((char *)(&FloatSend) + 1);
		U8Send[1] = *((char *)(&FloatSend) + 2);
		U8Send[0] = *((char *)(&FloatSend) + 3);
}