#include "Cal_V_Angular.h"

/**
 * @brief 初始化MTi解析器
 * @param parser 解析器指针
 */
void MTI_Parser_Init(MTI_Parser_t* parser)
{
    if (parser == NULL) return;
    
    parser->state = MTI_PARSE_STATE_WAIT_PREAMBLE;
    parser->buffer_index = 0;
    parser->bid = 0;
    parser->mid = 0;
    parser->len = 0;
    parser->checksum = 0;
    parser->error_count = 0;
    parser->success_count = 0;
    
    memset(parser->buffer, 0, sizeof(parser->buffer));
}

/**
 * @brief 重置解析器状态
 * @param parser 解析器指针
 */
void MTI_Parser_Reset(MTI_Parser_t* parser)
{
    if (parser == NULL) return;
    
    parser->state = MTI_PARSE_STATE_WAIT_PREAMBLE;
    parser->buffer_index = 0;
}

/**
 * @brief 处理单个字节数据
 * @param parser 解析器指针
 * @param byte 输入字节
 * @param imu_data 输出IMU数据
 * @retval 1-解析成功, 0-继续解析
 */
uint8_t MTI_Parser_ProcessByte(MTI_Parser_t* parser, uint8_t byte, MTI_IMU_Data_t* imu_data)
{
    if (parser == NULL) return 0;
    
    switch (parser->state) {
        case MTI_PARSE_STATE_WAIT_PREAMBLE:
            if (byte == MTI_PREAMBLE) {
                parser->buffer[0] = byte;
                parser->buffer_index = 1;
                parser->state = MTI_PARSE_STATE_WAIT_BID;
            }
            break;
            
        case MTI_PARSE_STATE_WAIT_BID:
            parser->bid = byte;
            parser->buffer[parser->buffer_index++] = byte;
            parser->state = MTI_PARSE_STATE_WAIT_MID;
            break;
            
        case MTI_PARSE_STATE_WAIT_MID:
            parser->mid = byte;
            parser->buffer[parser->buffer_index++] = byte;
            parser->state = MTI_PARSE_STATE_WAIT_LEN;
            break;
            
        case MTI_PARSE_STATE_WAIT_LEN:
            parser->len = byte;
            parser->buffer[parser->buffer_index++] = byte;
            
            if (parser->len == 0) {
                parser->state = MTI_PARSE_STATE_WAIT_CHECKSUM;
            } else if (parser->len > MTI_MAX_MSG_LEN - MTI_HEADER_LEN - MTI_CHECKSUM_LEN) {
                // 数据长度异常，重置解析器
                MTI_Parser_Reset(parser);
                parser->error_count++;
            } else {
                parser->state = MTI_PARSE_STATE_WAIT_DATA;
            }
            break;
            
        case MTI_PARSE_STATE_WAIT_DATA:
            parser->buffer[parser->buffer_index++] = byte;
            
            if (parser->buffer_index >= (MTI_HEADER_LEN + parser->len)) {
                parser->state = MTI_PARSE_STATE_WAIT_CHECKSUM;
            }
            break;
            
        case MTI_PARSE_STATE_WAIT_CHECKSUM:
            parser->checksum = byte;
            parser->buffer[parser->buffer_index++] = byte;
            
            // 验证校验和
            uint8_t calculated_checksum = MTI_Calculate_Checksum(parser->buffer, 
                                                               parser->buffer_index - 1);
            
            if (calculated_checksum == parser->checksum) {
                // 校验成功，解析数据
                if (parser->mid == MTI_MID_MTDATA2 && imu_data != NULL) {
                    uint8_t parse_result = MTI_Parse_MTData2(&parser->buffer[MTI_HEADER_LEN], 
                                                           parser->len, imu_data);
                    if (parse_result) {
                        parser->success_count++;
                        MTI_Parser_Reset(parser);
                        return 1;  // 解析成功
                    }
                }
            } else {
                parser->error_count++;
            }
            
            MTI_Parser_Reset(parser);
            break;
            
        default:
            MTI_Parser_Reset(parser);
            break;
    }
    
    return 0;  // 继续解析
}

/**
 * @brief 解析MTData2消息
 * @param data 数据缓冲区
 * @param len 数据长度
 * @param imu_data 输出IMU数据
 * @retval 1-成功, 0-失败
 */
uint8_t MTI_Parse_MTData2(const uint8_t* data, uint8_t len, MTI_IMU_Data_t* imu_data)
{
    if (data == NULL || imu_data == NULL || len < 4) return 0;
    
    uint8_t data_updated = 0;
    uint16_t index = 0;
    
    // 清除更新标志
    imu_data->euler.valid = 0;
    imu_data->gyro.valid = 0;
    
    while (index < len - 3) {
        // 读取数据标识符(XDI) - 2字节，大端序
        uint16_t xdi = MTI_Bytes_To_Uint16(&data[index]);
        index += 2;
        
        // 读取数据长度 - 1字节
        uint8_t data_len = data[index++];
        
        // 检查数据长度是否合理
        if (index + data_len > len) {
            break;  // 数据不完整
        }
        
        switch (xdi) {
            case MTI_XDI_EULER_ANGLES:
                if (data_len == MTI_EULER_DATA_LEN) {
                    // 解析欧拉角 (Roll, Pitch, Yaw) - 弧度转度
                    imu_data->euler.roll = MTI_Bytes_To_Float(&data[index]) * 180.0f / M_PI;
                    imu_data->euler.pitch = MTI_Bytes_To_Float(&data[index + 4]) * 180.0f / M_PI;
                    imu_data->euler.yaw = MTI_Bytes_To_Float(&data[index + 8]) * 180.0f / M_PI;
                    imu_data->euler.valid = 1;
                    imu_data->euler.timestamp = HAL_GetTick();
                    data_updated = 1;
                    
                    // 触发回调
                    MTI_EulerAngles_Callback(&imu_data->euler);
                }
                break;
                
            case MTI_XDI_RATE_OF_TURN:
                if (data_len == MTI_GYRO_DATA_LEN) {
                    // 解析角速度 (Gyr_X, Gyr_Y, Gyr_Z) - 弧度/秒转度/秒
                    imu_data->gyro.gyro_x = MTI_Bytes_To_Float(&data[index]) * 180.0f / M_PI;
                    imu_data->gyro.gyro_y = MTI_Bytes_To_Float(&data[index + 4]) * 180.0f / M_PI;
                    imu_data->gyro.gyro_z = MTI_Bytes_To_Float(&data[index + 8]) * 180.0f / M_PI;
                    imu_data->gyro.valid = 1;
                    imu_data->gyro.timestamp = HAL_GetTick();
                    data_updated = 1;
                    
                    // 触发回调
                    MTI_GyroData_Callback(&imu_data->gyro);
                }
                break;
                
            case MTI_XDI_PACKET_COUNTER:
                if (data_len == 2) {
                    imu_data->packet_counter = MTI_Bytes_To_Uint16(&data[index]);
                }
                break;
                
            // 可以在这里添加其他数据类型的解析
            default:
                // 未知数据类型，跳过
                break;
        }
        
        index += data_len;
    }
    
    if (data_updated) {
        imu_data->data_updated = 1;
        // 触发完整数据回调
        MTI_IMU_Data_Callback(imu_data);
    }
    
    return data_updated;
}

/**
 * @brief 计算校验和
 * @param data 数据缓冲区
 * @param len 数据长度
 * @retval 校验和值
 */
uint8_t MTI_Calculate_Checksum(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len < MTI_HEADER_LEN) return 0;
    
    uint8_t checksum = 0;
    
    // 跳过前导码，从BID开始计算
    for (uint16_t i = 1; i < len; i++) {
        checksum += data[i];
    }
    
    return (0x100 - checksum) & 0xFF;
}

/**
 * @brief 将4字节数据转换为float (IEEE 754, 大端序)
 * @param bytes 4字节数据
 * @retval float值
 */
float MTI_Bytes_To_Float(const uint8_t* bytes)
{
    if (bytes == NULL) return 0.0f;
    
    union {
        uint32_t i;
        float f;
    } converter;
    
    // 大端序转换
    converter.i = ((uint32_t)bytes[0] << 24) |
                  ((uint32_t)bytes[1] << 16) |
                  ((uint32_t)bytes[2] << 8) |
                  ((uint32_t)bytes[3]);
    
    return converter.f;
}

/**
 * @brief 将2字节数据转换为uint16 (大端序)
 * @param bytes 2字节数据
 * @retval uint16值
 */
uint16_t MTI_Bytes_To_Uint16(const uint8_t* bytes)
{
    if (bytes == NULL) return 0;
    
    return ((uint16_t)bytes[0] << 8) | ((uint16_t)bytes[1]);
}

// 弱函数实现，用户可重写
__weak void MTI_EulerAngles_Callback(MTI_EulerAngles_t* euler)
{
    // 用户可重写此函数处理欧拉角数据
    UNUSED(euler);
}

__weak void MTI_GyroData_Callback(MTI_GyroData_t* gyro)
{
    // 用户可重写此函数处理角速度数据
    UNUSED(gyro);
}

__weak void MTI_IMU_Data_Callback(MTI_IMU_Data_t* imu_data)
{
    // 用户可重写此函数处理完整IMU数据
    UNUSED(imu_data);
}
