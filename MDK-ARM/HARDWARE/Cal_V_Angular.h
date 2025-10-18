#ifndef ATTITUDE_ANGULAR_VELOCITY_H
#define ATTITUDE_ANGULAR_VELOCITY_H


#include "main.h"
#include <stdint.h>
#include <string.h>

#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif
#define M_2PI                   (2.0f * M_PI)
#define DEG_TO_RAD              (M_PI / 180.0f)
#define RAD_TO_DEG              (180.0f / M_PI)


// MTiЭ�鳣��
#define MTI_PREAMBLE                0xFA    // ��Ϣǰ����
#define MTI_BID_MASTER              0xFF    // ���豸ID
#define MTI_MAX_MSG_LEN             256     // �����Ϣ����
#define MTI_HEADER_LEN              4       // ��Ϣͷ����
#define MTI_CHECKSUM_LEN            1       // У��ͳ���

// ��Ҫ��ϢID
#define MTI_MID_MTDATA2             0x36    // MTData2��Ϣ

// MTData2��������ID
#define MTI_XDI_EULER_ANGLES        0x2030  // ŷ���� (Roll, Pitch, Yaw)
#define MTI_XDI_RATE_OF_TURN        0x8020  // ���ٶ� (Gyr_X, Gyr_Y, Gyr_Z)
#define MTI_XDI_ACCELERATION        0x4020  // ���ٶ�
#define MTI_XDI_MAGNETIC_FIELD      0xC020  // �ų�
#define MTI_XDI_QUATERNION          0x2010  // ��Ԫ��
#define MTI_XDI_PACKET_COUNTER      0x1020  // ��������
#define MTI_XDI_UTC_TIME            0x1010  // UTCʱ��

// ���ݳ��ȶ���
#define MTI_EULER_DATA_LEN          12      // ŷ�������ݳ��� (3 * float)
#define MTI_GYRO_DATA_LEN           12      // ���ٶ����ݳ��� (3 * float)
#define MTI_ACCEL_DATA_LEN          12      // ���ٶ����ݳ���
#define MTI_QUAT_DATA_LEN           16      // ��Ԫ�����ݳ��� (4 * float)

// ����״̬
typedef enum {
    MTI_PARSE_STATE_WAIT_PREAMBLE = 0,
    MTI_PARSE_STATE_WAIT_BID,
    MTI_PARSE_STATE_WAIT_MID,
    MTI_PARSE_STATE_WAIT_LEN,
    MTI_PARSE_STATE_WAIT_DATA,
    MTI_PARSE_STATE_WAIT_CHECKSUM
} MTI_ParseState_t;

// ŷ�������ݽṹ
typedef struct {
    float roll;         // ��ת�� (��)
    float pitch;        // ������ (��)
    float yaw;          // ƫ���� (��)
    uint8_t valid;      // ������Ч��־
    uint32_t timestamp; // ʱ���
} MTI_EulerAngles_t;

// ���ٶ����ݽṹ
typedef struct {
    float gyro_x;       // X����ٶ� (��/��)
    float gyro_y;       // Y����ٶ� (��/��)
    float gyro_z;       // Z����ٶ� (��/��)
    uint8_t valid;      // ������Ч��־
    uint32_t timestamp; // ʱ���
} MTI_GyroData_t;

// ����IMU���ݽṹ
typedef struct {
    MTI_EulerAngles_t euler;    // ŷ����
    MTI_GyroData_t gyro;        // ���ٶ�
    uint16_t packet_counter;    // ��������
    uint8_t data_updated;       // ���ݸ��±�־
} MTI_IMU_Data_t;

// MTi�������ṹ
typedef struct {
    MTI_ParseState_t state;     // ����״̬
    uint8_t buffer[MTI_MAX_MSG_LEN];  // ���ջ�����
    uint16_t buffer_index;      // ����������
    uint8_t bid;                // �豸ID
    uint8_t mid;                // ��ϢID
    uint8_t len;                // ���ݳ���
    uint8_t checksum;           // У���
    uint32_t error_count;       // �������
    uint32_t success_count;     // �ɹ���������
} MTI_Parser_t;

// ��������
void MTI_Parser_Init(MTI_Parser_t* parser);
void MTI_Parser_Reset(MTI_Parser_t* parser);
uint8_t MTI_Parser_ProcessByte(MTI_Parser_t* parser, uint8_t byte, MTI_IMU_Data_t* imu_data);
uint8_t MTI_Parse_MTData2(const uint8_t* data, uint8_t len, MTI_IMU_Data_t* imu_data);
uint8_t MTI_Calculate_Checksum(const uint8_t* data, uint16_t len);
float MTI_Bytes_To_Float(const uint8_t* bytes);
uint16_t MTI_Bytes_To_Uint16(const uint8_t* bytes);

// �ص����� (�û�����д)
void MTI_EulerAngles_Callback(MTI_EulerAngles_t* euler);
void MTI_GyroData_Callback(MTI_GyroData_t* gyro);
void MTI_IMU_Data_Callback(MTI_IMU_Data_t* imu_data);

#endif

