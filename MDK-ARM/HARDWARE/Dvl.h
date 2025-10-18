#ifndef _DVL_H_
#define _DVL_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define DVL_LINE_SIZE  64

typedef struct {
    char line_buffer[DVL_LINE_SIZE];
    uint16_t line_index;
    
    float vx, vy, vz;
    bool updated;
} DVL_Parser_t;

void DVL_Init(DVL_Parser_t *parser, UART_HandleTypeDef *huart);
void DVL_RxCallback(DVL_Parser_t *parser, uint8_t byte);
bool DVL_GetData(DVL_Parser_t *parser, float *x, float *y, float *z);

#endif

