#include "Cal_V_Angular.h"

/**
 * @brief ��ʼ��MTi������
 * @param parser ������ָ��
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
 * @brief ���ý�����״̬
 * @param parser ������ָ��
 */
void MTI_Parser_Reset(MTI_Parser_t* parser)
{
    if (parser == NULL) return;
    
    parser->state = MTI_PARSE_STATE_WAIT_PREAMBLE;
    parser->buffer_index = 0;
}

/**
 * @brief �������ֽ�����
 * @param parser ������ָ��
 * @param byte �����ֽ�
 * @param imu_data ���IMU����
 * @retval 1-�����ɹ�, 0-��������
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
                // ���ݳ����쳣�����ý�����
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
            
            // ��֤У���
            uint8_t calculated_checksum = MTI_Calculate_Checksum(parser->buffer, 
                                                               parser->buffer_index - 1);
            
            if (calculated_checksum == parser->checksum) {
                // У��ɹ�����������
                if (parser->mid == MTI_MID_MTDATA2 && imu_data != NULL) {
                    uint8_t parse_result = MTI_Parse_MTData2(&parser->buffer[MTI_HEADER_LEN], 
                                                           parser->len, imu_data);
                    if (parse_result) {
                        parser->success_count++;
                        MTI_Parser_Reset(parser);
                        return 1;  // �����ɹ�
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
    
    return 0;  // ��������
}

/**
 * @brief ����MTData2��Ϣ
 * @param data ���ݻ�����
 * @param len ���ݳ���
 * @param imu_data ���IMU����
 * @retval 1-�ɹ�, 0-ʧ��
 */
uint8_t MTI_Parse_MTData2(const uint8_t* data, uint8_t len, MTI_IMU_Data_t* imu_data)
{
    if (data == NULL || imu_data == NULL || len < 4) return 0;
    
    uint8_t data_updated = 0;
    uint16_t index = 0;
    
    // ������±�־
    imu_data->euler.valid = 0;
    imu_data->gyro.valid = 0;
    
    while (index < len - 3) {
        // ��ȡ���ݱ�ʶ��(XDI) - 2�ֽڣ������
        uint16_t xdi = MTI_Bytes_To_Uint16(&data[index]);
        index += 2;
        
        // ��ȡ���ݳ��� - 1�ֽ�
        uint8_t data_len = data[index++];
        
        // ������ݳ����Ƿ����
        if (index + data_len > len) {
            break;  // ���ݲ�����
        }
        
        switch (xdi) {
            case MTI_XDI_EULER_ANGLES:
                if (data_len == MTI_EULER_DATA_LEN) {
                    // ����ŷ���� (Roll, Pitch, Yaw) - ����ת��
                    imu_data->euler.roll = MTI_Bytes_To_Float(&data[index]) * 180.0f / M_PI;
                    imu_data->euler.pitch = MTI_Bytes_To_Float(&data[index + 4]) * 180.0f / M_PI;
                    imu_data->euler.yaw = MTI_Bytes_To_Float(&data[index + 8]) * 180.0f / M_PI;
                    imu_data->euler.valid = 1;
                    imu_data->euler.timestamp = HAL_GetTick();
                    data_updated = 1;
                    
                    // �����ص�
                    MTI_EulerAngles_Callback(&imu_data->euler);
                }
                break;
                
            case MTI_XDI_RATE_OF_TURN:
                if (data_len == MTI_GYRO_DATA_LEN) {
                    // �������ٶ� (Gyr_X, Gyr_Y, Gyr_Z) - ����/��ת��/��
                    imu_data->gyro.gyro_x = MTI_Bytes_To_Float(&data[index]) * 180.0f / M_PI;
                    imu_data->gyro.gyro_y = MTI_Bytes_To_Float(&data[index + 4]) * 180.0f / M_PI;
                    imu_data->gyro.gyro_z = MTI_Bytes_To_Float(&data[index + 8]) * 180.0f / M_PI;
                    imu_data->gyro.valid = 1;
                    imu_data->gyro.timestamp = HAL_GetTick();
                    data_updated = 1;
                    
                    // �����ص�
                    MTI_GyroData_Callback(&imu_data->gyro);
                }
                break;
                
            case MTI_XDI_PACKET_COUNTER:
                if (data_len == 2) {
                    imu_data->packet_counter = MTI_Bytes_To_Uint16(&data[index]);
                }
                break;
                
            // ������������������������͵Ľ���
            default:
                // δ֪�������ͣ�����
                break;
        }
        
        index += data_len;
    }
    
    if (data_updated) {
        imu_data->data_updated = 1;
        // �����������ݻص�
        MTI_IMU_Data_Callback(imu_data);
    }
    
    return data_updated;
}

/**
 * @brief ����У���
 * @param data ���ݻ�����
 * @param len ���ݳ���
 * @retval У���ֵ
 */
uint8_t MTI_Calculate_Checksum(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len < MTI_HEADER_LEN) return 0;
    
    uint8_t checksum = 0;
    
    // ����ǰ���룬��BID��ʼ����
    for (uint16_t i = 1; i < len; i++) {
        checksum += data[i];
    }
    
    return (0x100 - checksum) & 0xFF;
}

/**
 * @brief ��4�ֽ�����ת��Ϊfloat (IEEE 754, �����)
 * @param bytes 4�ֽ�����
 * @retval floatֵ
 */
float MTI_Bytes_To_Float(const uint8_t* bytes)
{
    if (bytes == NULL) return 0.0f;
    
    union {
        uint32_t i;
        float f;
    } converter;
    
    // �����ת��
    converter.i = ((uint32_t)bytes[0] << 24) |
                  ((uint32_t)bytes[1] << 16) |
                  ((uint32_t)bytes[2] << 8) |
                  ((uint32_t)bytes[3]);
    
    return converter.f;
}

/**
 * @brief ��2�ֽ�����ת��Ϊuint16 (�����)
 * @param bytes 2�ֽ�����
 * @retval uint16ֵ
 */
uint16_t MTI_Bytes_To_Uint16(const uint8_t* bytes)
{
    if (bytes == NULL) return 0;
    
    return ((uint16_t)bytes[0] << 8) | ((uint16_t)bytes[1]);
}

// ������ʵ�֣��û�����д
__weak void MTI_EulerAngles_Callback(MTI_EulerAngles_t* euler)
{
    // �û�����д�˺�������ŷ��������
    UNUSED(euler);
}

__weak void MTI_GyroData_Callback(MTI_GyroData_t* gyro)
{
    // �û�����д�˺���������ٶ�����
    UNUSED(gyro);
}

__weak void MTI_IMU_Data_Callback(MTI_IMU_Data_t* imu_data)
{
    // �û�����д�˺�����������IMU����
    UNUSED(imu_data);
}
