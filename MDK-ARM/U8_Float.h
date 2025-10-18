#ifndef _U8_Float_H_
#define _U8_Float_H_

#include "main.h"

extern uint8_t U8Send[4];
extern uint8_t U8Data[4];
extern float FloatSend;

float U8ToFloat(uint8_t U8Data[4]);
void FloatToU8(float FloatSend ,uint8_t U8Send[4]);

#endif