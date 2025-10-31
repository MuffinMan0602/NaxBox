#include "Navigation.h"
#include <string.h>
#include <math.h>

// ��ʼ��
void DR_Init(DeadReckoning_t* dr)
{
    if (dr == NULL) return;
    
    memset(dr, 0, sizeof(DeadReckoning_t));
    
    // Ĭ������
    dr->config.use_trapezoidal = 1;
    dr->config.use_interpolation = 1;
    dr->config.use_attitude_compensation = 1;
    dr->config.velocity_threshold = 0.01f;  // 1cm/s
    dr->config.max_dt = 0.2f;               // 200ms
}

// ��������
void DR_SetConfig(DeadReckoning_t* dr, const IntegratorConfig_t* config)
{
    if (dr == NULL || config == NULL) return;
    dr->config = *config;
}

// ����λ��
void DR_Reset(DeadReckoning_t* dr, float init_x, float init_y, float init_z)
{
    if (dr == NULL) return;
    
    dr->position.x = init_x;
    dr->position.y = init_y;
    dr->position.z = init_z;
    
    memset(&dr->velocity_earth_current, 0, sizeof(Position_t));
    memset(&dr->velocity_earth_previous, 0, sizeof(Position_t));
    
    dr->vel_buffer.count = 0;
    dr->att_buffer.count = 0;
    dr->initialized = 0;
}

// ���Բ�ֵ
float DR_LinearInterp(float t, float t1, float t2, float v1, float v2)
{
    if (t2 <= t1) return v1;
    
    float ratio = (t - t1) / (t2 - t1);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    
    return v1 + ratio * (v2 - v1);
}

// �ٶȲ�ֵ
void DR_InterpolateVelocity(const VelocityBuffer_t* buffer, uint32_t target_time, Velocity_t* result)
{
    if (buffer == NULL || result == NULL || buffer->count == 0) {
        memset(result, 0, sizeof(Velocity_t));
        return;
    }
    
    if (buffer->count == 1) {
        *result = buffer->velocity[0];
        return;
    }
    
    // �ҵ����ʵĲ�ֵ����
    int idx1 = -1, idx2 = -1;
    
    for (int i = 0; i < buffer->count - 1; i++) {
        int curr_idx = (buffer->index - i + 3) % 3;
        int next_idx = (buffer->index - i - 1 + 3) % 3;
        
        if (buffer->timestamp[next_idx] <= target_time && 
            target_time <= buffer->timestamp[curr_idx]) {
            idx1 = next_idx;
            idx2 = curr_idx;
            break;
        }
    }
    
    // ���û�ҵ��������䣬ʹ����������
    if (idx1 == -1 || idx2 == -1) {
        *result = buffer->velocity[buffer->index];
        return;
    }
    
    // ���Բ�ֵ
    float t = (float)target_time;
    float t1 = (float)buffer->timestamp[idx1];
    float t2 = (float)buffer->timestamp[idx2];
    
    result->u = DR_LinearInterp(t, t1, t2, 
                               buffer->velocity[idx1].u, 
                               buffer->velocity[idx2].u);
    result->v = DR_LinearInterp(t, t1, t2, 
                               buffer->velocity[idx1].v, 
                               buffer->velocity[idx2].v);
    result->w = DR_LinearInterp(t, t1, t2, 
                               buffer->velocity[idx1].w, 
                               buffer->velocity[idx2].w);
}

// ��̬��ֵ������Ƕ����䣩
void DR_InterpolateAttitude(const AttitudeBuffer_t* buffer, uint32_t target_time, Attitude_t* result)
{
    if (buffer == NULL || result == NULL || buffer->count == 0) {
        memset(result, 0, sizeof(Attitude_t));
        return;
    }
    
    if (buffer->count == 1) {
        *result = buffer->attitude[0];
        return;
    }
    
    // �ҵ���ֵ����
    int idx1 = -1, idx2 = -1;
    
    for (int i = 0; i < buffer->count - 1; i++) {
        int curr_idx = (buffer->index - i + 5) % 5;
        int next_idx = (buffer->index - i - 1 + 5) % 5;
        
        if (buffer->timestamp[next_idx] <= target_time && 
            target_time <= buffer->timestamp[curr_idx]) {
            idx1 = next_idx;
            idx2 = curr_idx;
            break;
        }
    }
    
    if (idx1 == -1 || idx2 == -1) {
        *result = buffer->attitude[buffer->index];
        return;
    }
    
    // �ǶȲ�ֵ�����������ԣ�
    float t = (float)target_time;
    float t1 = (float)buffer->timestamp[idx1];
    float t2 = (float)buffer->timestamp[idx2];
    
    result->roll = DR_LinearInterp(t, t1, t2, 
                                  buffer->attitude[idx1].roll, 
                                  buffer->attitude[idx2].roll);
    result->pitch = DR_LinearInterp(t, t1, t2, 
                                   buffer->attitude[idx1].pitch, 
                                   buffer->attitude[idx2].pitch);
    
    // yaw�Ǵ���������
    float yaw1 = buffer->attitude[idx1].yaw;
    float yaw2 = buffer->attitude[idx2].yaw;
    float yaw_diff = yaw2 - yaw1;
    
    // ����-�е��е�����
    if (yaw_diff > M_PI) {
        yaw2 -= 2 * M_PI;
    } else if (yaw_diff < -M_PI) {
        yaw2 += 2 * M_PI;
    }
    
    result->yaw = DR_LinearInterp(t, t1, t2, yaw1, yaw2);
    
    // ȷ��yaw��[-��, ��]��Χ��
    while (result->yaw > M_PI) result->yaw -= 2 * M_PI;
    while (result->yaw < -M_PI) result->yaw += 2 * M_PI;
}

// ����ת��
void DR_BodyToEarth(const Velocity_t* body_vel, const Attitude_t* attitude, Position_t* earth_vel)
{
    if (body_vel == NULL || attitude == NULL || earth_vel == NULL) return;
    
    float cos_roll = cosf(attitude->roll);
    float sin_roll = sinf(attitude->roll);
    float cos_pitch = cosf(attitude->pitch);
    float sin_pitch = sinf(attitude->pitch);
    float cos_yaw = cosf(attitude->yaw);
    float sin_yaw = sinf(attitude->yaw);
    
    // ��ת����
    float r11 = cos_yaw * cos_pitch;
    float r12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    float r13 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    
    float r21 = sin_yaw * cos_pitch;
    float r22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    float r23 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    
    float r31 = -sin_pitch;
    float r32 = cos_pitch * sin_roll;
    float r33 = cos_pitch * cos_roll;
    
    earth_vel->x = r11 * body_vel->u + r12 * body_vel->v + r13 * body_vel->w;
    earth_vel->y = r21 * body_vel->u + r22 * body_vel->v + r23 * body_vel->w;
    earth_vel->z = r31 * body_vel->u + r32 * body_vel->v + r33 * body_vel->w;
}

// �����ٶ�����
void DR_UpdateVelocity(DeadReckoning_t* dr, const Velocity_t* body_vel, uint32_t timestamp)
{
    if (dr == NULL || body_vel == NULL) return;
    
    // �����ٶȻ���
    dr->vel_buffer.index = (dr->vel_buffer.index + 1) % 3;
    dr->vel_buffer.velocity[dr->vel_buffer.index] = *body_vel;
    dr->vel_buffer.timestamp[dr->vel_buffer.index] = timestamp;
    
    if (dr->vel_buffer.count < 3) {
        dr->vel_buffer.count++;
    }
    
    dr->last_velocity_timestamp = timestamp;
}

// ������̬����
void DR_UpdateAttitude(DeadReckoning_t* dr, const Attitude_t* attitude, uint32_t timestamp)
{
    if (dr == NULL || attitude == NULL) return;
    
    // ������̬����
    dr->att_buffer.index = (dr->att_buffer.index + 1) % 5;
    dr->att_buffer.attitude[dr->att_buffer.index] = *attitude;
    dr->att_buffer.timestamp[dr->att_buffer.index] = timestamp;
    
    if (dr->att_buffer.count < 5) {
        dr->att_buffer.count++;
    }
    
    dr->last_attitude_timestamp = timestamp;
}

// �������
void DR_UpdateDepth(DeadReckoning_t* dr, float depth)
{
    if (dr == NULL) return;
    dr->position.z = depth;
}

// ��Ƶλ�ø���
void DR_UpdatePosition(DeadReckoning_t* dr, uint32_t timestamp)
{
    if (dr == NULL) return;
    
    // ����Ƿ����㹻������
    if (dr->vel_buffer.count == 0 || dr->att_buffer.count == 0) {
        return;
    }
    
    // �״γ�ʼ��
    if (!dr->initialized) {
        dr->last_update_timestamp = timestamp;
        dr->initialized = 1;
        
        // �����ʼ�������ϵ�ٶ�
        Velocity_t current_body_vel;
        Attitude_t current_attitude;
        
        if (dr->config.use_interpolation) {
            DR_InterpolateVelocity(&dr->vel_buffer, timestamp, &current_body_vel);
            DR_InterpolateAttitude(&dr->att_buffer, timestamp, &current_attitude);
        } else {
            current_body_vel = dr->vel_buffer.velocity[dr->vel_buffer.index];
            current_attitude = dr->att_buffer.attitude[dr->att_buffer.index];
        }
        
        DR_BodyToEarth(&current_body_vel, &current_attitude, &dr->velocity_earth_current);
        dr->velocity_earth_previous = dr->velocity_earth_current;
        return;
    }
    
    // ����ʱ����
    float dt = (float)(timestamp - dr->last_update_timestamp) / 1000.0f;
    
    // ����ʱ����
    if (dt <= 0.0f || dt > dr->config.max_dt) {
        dr->last_update_timestamp = timestamp;
        return;
    }
    
    // ��ȡ��ǰʱ�̵��ٶȺ���̬����ֵ��ֱ��ʹ�ã�
    Velocity_t current_body_vel;
    Attitude_t current_attitude;
    
    if (dr->config.use_interpolation) {
        DR_InterpolateVelocity(&dr->vel_buffer, timestamp, &current_body_vel);
        DR_InterpolateAttitude(&dr->att_buffer, timestamp, &current_attitude);
    } else {
        current_body_vel = dr->vel_buffer.velocity[dr->vel_buffer.index];
        current_attitude = dr->att_buffer.attitude[dr->att_buffer.index];
    }
    
    // ת�����������ϵ
    DR_BodyToEarth(&current_body_vel, &current_attitude, &dr->velocity_earth_current);
    
    // �ٶ���ֵ����
    if (sqrtf(dr->velocity_earth_current.x * dr->velocity_earth_current.x + 
              dr->velocity_earth_current.y * dr->velocity_earth_current.y) < 
        dr->config.velocity_threshold) {
        dr->velocity_earth_current.x = 0.0f;
        dr->velocity_earth_current.y = 0.0f;
    }
    
    // λ�û���
    if (dr->config.use_trapezoidal) {
        // ���λ��֣���׼ȷ
        dr->position.x += 0.5f * (dr->velocity_earth_current.x + dr->velocity_earth_previous.x) * dt;
        dr->position.y += 0.5f * (dr->velocity_earth_current.y + dr->velocity_earth_previous.y) * dt;
    } else {
        // ��ŷ������
        dr->position.x += dr->velocity_earth_current.x * dt;
        dr->position.y += dr->velocity_earth_current.y * dt;
    }
    
    // ���浱ǰ�ٶ���Ϊ�´ε�ǰһ�ٶ�
    dr->velocity_earth_previous = dr->velocity_earth_current;
    
    // ����ʱ���
    dr->last_update_timestamp = timestamp;
}

// ��ȡλ��
void DR_GetPosition(const DeadReckoning_t* dr, Position_t* pos)
{
    if (dr == NULL || pos == NULL) return;
    *pos = dr->position;
}

// ��ȡ�������ϵ�ٶ�
void DR_GetVelocityEarth(const DeadReckoning_t* dr, Position_t* vel)
{
    if (dr == NULL || vel == NULL) return;
    *vel = dr->velocity_earth_current;
}
