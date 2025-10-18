#include "Dvl.h"

#include <string.h>
#include <stdlib.h>

static void parse_line(DVL_Parser_t *parser) {
    if (strncmp(parser->line_buffer, ":BI,", 4) != 0) return;
    
    char *data = parser->line_buffer + 4;
    char *token;
    
    token = strtok(data, ",");
    if (!token) return;
    parser->vx = atof(token);
    
    token = strtok(NULL, ",");
    if (!token) return;
    parser->vy = atof(token);
    
    token = strtok(NULL, ",");
    if (!token) return;
    parser->vz = atof(token);
    
    parser->updated = true;
}

void DVL_Init(DVL_Parser_t *parser, UART_HandleTypeDef *huart) {
    memset(parser, 0, sizeof(DVL_Parser_t));
}

void DVL_RxCallback(DVL_Parser_t *parser, uint8_t byte) {
    if (byte == '\n') {
        if (parser->line_index > 0) {
            parser->line_buffer[parser->line_index] = '\0';
            parse_line(parser);
            parser->line_index = 0;
        }
    } else if (byte != '\r' && parser->line_index < DVL_LINE_SIZE - 1) {
        parser->line_buffer[parser->line_index++] = byte;
    }
}

bool DVL_GetData(DVL_Parser_t *parser, float *x, float *y, float *z) {
    if (parser->updated) {
        *x = parser->vx;
        *y = parser->vy;
        *z = parser->vz;
        parser->updated = false;
        return true;
    }
    return false;
}
