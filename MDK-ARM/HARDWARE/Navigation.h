#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
// 手动定义数学常数
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// 数据结构定义
typedef struct {
    float x, y, z;
} Position_t;

typedef struct {
    float u, v, w;
} Velocity_t;

typedef struct {
    float roll, pitch, yaw;
} Attitude_t;

// 速度插值缓存
typedef struct {
    Velocity_t velocity[3];      // 保存最近3个速度值
    uint32_t timestamp[3];       // 对应的时间戳
    uint8_t index;               // 当前索引
    uint8_t count;               // 有效数据个数
} VelocityBuffer_t;

// 姿态插值缓存
typedef struct {
    Attitude_t attitude[5];      // 保存最近5个姿态值
    uint32_t timestamp[5];       // 对应的时间戳
    uint8_t index;               // 当前索引
    uint8_t count;               // 有效数据个数
} AttitudeBuffer_t;

// 积分器配置
typedef struct {
    uint8_t use_trapezoidal;     // 使用梯形积分
    uint8_t use_interpolation;   // 使用插值
    uint8_t use_attitude_compensation; // 使用姿态补偿
    float velocity_threshold;    // 速度阈值，低于此值认为静止
    float max_dt;                // 最大允许时间间隔(s)
} IntegratorConfig_t;

typedef struct {
    Position_t position;
    Position_t velocity_earth_current;
    Position_t velocity_earth_previous;  // 上一时刻的大地速度
    
    VelocityBuffer_t vel_buffer;
    AttitudeBuffer_t att_buffer;
    
    uint32_t last_velocity_timestamp;
    uint32_t last_attitude_timestamp;
    uint32_t last_update_timestamp;
    
    uint8_t initialized;
    IntegratorConfig_t config;
} DeadReckoning_t;

// 函数声明
void DR_Init(DeadReckoning_t* dr);
void DR_Reset(DeadReckoning_t* dr, float init_x, float init_y, float init_z);
void DR_SetConfig(DeadReckoning_t* dr, const IntegratorConfig_t* config);

// 主要更新函数
void DR_UpdateVelocity(DeadReckoning_t* dr, const Velocity_t* body_vel, uint32_t timestamp);
void DR_UpdateAttitude(DeadReckoning_t* dr, const Attitude_t* attitude, uint32_t timestamp);
void DR_UpdateDepth(DeadReckoning_t* dr, float depth);

// 高频率位置更新（可在主循环中高频调用）
void DR_UpdatePosition(DeadReckoning_t* dr, uint32_t timestamp);

// 插值函数
void DR_InterpolateVelocity(const VelocityBuffer_t* buffer, uint32_t target_time, Velocity_t* result);
void DR_InterpolateAttitude(const AttitudeBuffer_t* buffer, uint32_t target_time, Attitude_t* result);

// 获取结果
void DR_GetPosition(const DeadReckoning_t* dr, Position_t* pos);
void DR_GetVelocityEarth(const DeadReckoning_t* dr, Position_t* vel);

#endif
