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


// MTi协议常量
#define MTI_PREAMBLE                0xFA    // 消息前导码
#define MTI_BID_MASTER              0xFF    // 主设备ID
#define MTI_MAX_MSG_LEN             256     // 最大消息长度
#define MTI_HEADER_LEN              4       // 消息头长度
#define MTI_CHECKSUM_LEN            1       // 校验和长度

// 主要消息ID
#define MTI_MID_MTDATA2             0x36    // MTData2消息

// MTData2数据类型ID
#define MTI_XDI_EULER_ANGLES        0x2030  // 欧拉角 (Roll, Pitch, Yaw)
#define MTI_XDI_RATE_OF_TURN        0x8020  // 角速度 (Gyr_X, Gyr_Y, Gyr_Z)
#define MTI_XDI_ACCELERATION        0x4020  // 加速度
#define MTI_XDI_MAGNETIC_FIELD      0xC020  // 磁场
#define MTI_XDI_QUATERNION          0x2010  // 四元数
#define MTI_XDI_PACKET_COUNTER      0x1020  // 包计数器
#define MTI_XDI_UTC_TIME            0x1010  // UTC时间

// 数据长度定义
#define MTI_EULER_DATA_LEN          12      // 欧拉角数据长度 (3 * float)
#define MTI_GYRO_DATA_LEN           12      // 角速度数据长度 (3 * float)
#define MTI_ACCEL_DATA_LEN          12      // 加速度数据长度
#define MTI_QUAT_DATA_LEN           16      // 四元数数据长度 (4 * float)

// 解析状态
typedef enum {
    MTI_PARSE_STATE_WAIT_PREAMBLE = 0,
    MTI_PARSE_STATE_WAIT_BID,
    MTI_PARSE_STATE_WAIT_MID,
    MTI_PARSE_STATE_WAIT_LEN,
    MTI_PARSE_STATE_WAIT_DATA,
    MTI_PARSE_STATE_WAIT_CHECKSUM
} MTI_ParseState_t;

// 欧拉角数据结构
typedef struct {
    float roll;         // 滚转角 (度)
    float pitch;        // 俯仰角 (度)
    float yaw;          // 偏航角 (度)
    uint8_t valid;      // 数据有效标志
    uint32_t timestamp; // 时间戳
} MTI_EulerAngles_t;

// 角速度数据结构
typedef struct {
    float gyro_x;       // X轴角速度 (度/秒)
    float gyro_y;       // Y轴角速度 (度/秒)
    float gyro_z;       // Z轴角速度 (度/秒)
    uint8_t valid;      // 数据有效标志
    uint32_t timestamp; // 时间戳
} MTI_GyroData_t;

// 完整IMU数据结构
typedef struct {
    MTI_EulerAngles_t euler;    // 欧拉角
    MTI_GyroData_t gyro;        // 角速度
    uint16_t packet_counter;    // 包计数器
    uint8_t data_updated;       // 数据更新标志
} MTI_IMU_Data_t;

// MTi解析器结构
typedef struct {
    MTI_ParseState_t state;     // 解析状态
    uint8_t buffer[MTI_MAX_MSG_LEN];  // 接收缓冲区
    uint16_t buffer_index;      // 缓冲区索引
    uint8_t bid;                // 设备ID
    uint8_t mid;                // 消息ID
    uint8_t len;                // 数据长度
    uint8_t checksum;           // 校验和
    uint32_t error_count;       // 错误计数
    uint32_t success_count;     // 成功解析计数
} MTI_Parser_t;

// 函数声明
void MTI_Parser_Init(MTI_Parser_t* parser);
void MTI_Parser_Reset(MTI_Parser_t* parser);
uint8_t MTI_Parser_ProcessByte(MTI_Parser_t* parser, uint8_t byte, MTI_IMU_Data_t* imu_data);
uint8_t MTI_Parse_MTData2(const uint8_t* data, uint8_t len, MTI_IMU_Data_t* imu_data);
uint8_t MTI_Calculate_Checksum(const uint8_t* data, uint16_t len);
float MTI_Bytes_To_Float(const uint8_t* bytes);
uint16_t MTI_Bytes_To_Uint16(const uint8_t* bytes);

// 回调函数 (用户可重写)
void MTI_EulerAngles_Callback(MTI_EulerAngles_t* euler);
void MTI_GyroData_Callback(MTI_GyroData_t* gyro);
void MTI_IMU_Data_Callback(MTI_IMU_Data_t* imu_data);

#endif

