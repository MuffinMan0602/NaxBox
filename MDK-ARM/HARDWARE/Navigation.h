#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
// �ֶ�������ѧ����
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// ���ݽṹ����
typedef struct {
    float x, y, z;
} Position_t;

typedef struct {
    float u, v, w;
} Velocity_t;

typedef struct {
    float roll, pitch, yaw;
} Attitude_t;

// �ٶȲ�ֵ����
typedef struct {
    Velocity_t velocity[3];      // �������3���ٶ�ֵ
    uint32_t timestamp[3];       // ��Ӧ��ʱ���
    uint8_t index;               // ��ǰ����
    uint8_t count;               // ��Ч���ݸ���
} VelocityBuffer_t;

// ��̬��ֵ����
typedef struct {
    Attitude_t attitude[5];      // �������5����ֵ̬
    uint32_t timestamp[5];       // ��Ӧ��ʱ���
    uint8_t index;               // ��ǰ����
    uint8_t count;               // ��Ч���ݸ���
} AttitudeBuffer_t;

// ����������
typedef struct {
    uint8_t use_trapezoidal;     // ʹ�����λ���
    uint8_t use_interpolation;   // ʹ�ò�ֵ
    uint8_t use_attitude_compensation; // ʹ����̬����
    float velocity_threshold;    // �ٶ���ֵ�����ڴ�ֵ��Ϊ��ֹ
    float max_dt;                // �������ʱ����(s)
} IntegratorConfig_t;

typedef struct {
    Position_t position;
    Position_t velocity_earth_current;
    Position_t velocity_earth_previous;  // ��һʱ�̵Ĵ���ٶ�
    
    VelocityBuffer_t vel_buffer;
    AttitudeBuffer_t att_buffer;
    
    uint32_t last_velocity_timestamp;
    uint32_t last_attitude_timestamp;
    uint32_t last_update_timestamp;
    
    uint8_t initialized;
    IntegratorConfig_t config;
} DeadReckoning_t;

// ��������
void DR_Init(DeadReckoning_t* dr);
void DR_Reset(DeadReckoning_t* dr, float init_x, float init_y, float init_z);
void DR_SetConfig(DeadReckoning_t* dr, const IntegratorConfig_t* config);

// ��Ҫ���º���
void DR_UpdateVelocity(DeadReckoning_t* dr, const Velocity_t* body_vel, uint32_t timestamp);
void DR_UpdateAttitude(DeadReckoning_t* dr, const Attitude_t* attitude, uint32_t timestamp);
void DR_UpdateDepth(DeadReckoning_t* dr, float depth);

// ��Ƶ��λ�ø��£�������ѭ���и�Ƶ���ã�
void DR_UpdatePosition(DeadReckoning_t* dr, uint32_t timestamp);

// ��ֵ����
void DR_InterpolateVelocity(const VelocityBuffer_t* buffer, uint32_t target_time, Velocity_t* result);
void DR_InterpolateAttitude(const AttitudeBuffer_t* buffer, uint32_t target_time, Attitude_t* result);

// ��ȡ���
void DR_GetPosition(const DeadReckoning_t* dr, Position_t* pos);
void DR_GetVelocityEarth(const DeadReckoning_t* dr, Position_t* vel);

#endif
