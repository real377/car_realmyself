#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数
    float integral;            // 积分累加量
    float integral_limit;      // 积分饱和上限（对称）
    float output_limit;        // 输出限幅（对称）
    float prev_error;          // 上次测量值（用于微分）
    float d_filtered;          // 微分项一阶低通滤波输出
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit, float o_limit);
float pid_update(pid_t *pid, float setpoint, float measured, float dt);

#endif