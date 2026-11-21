#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit, float o_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = (i_limit > 0.0f) ? i_limit : 0.0f;
    pid->output_limit   = (o_limit > 0.0f) ? o_limit : 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;     // 新增：滤波输出初始为零
}

float pid_update(pid_t *pid, float setpoint, float measured, float dt)
{
    // 1. 误差
    float error = setpoint - measured;

    // 2. 比例项
    float p_out = pid->kp * error;

    // 3. 积分项（带抗饱和）
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->ki * pid->integral;

    // 4. 微分项（测量值微分） + 低通滤波 (α = 0.3)
    float d_out = 0.0f;
    if (dt > 1e-6f) {
        float deriv = -(measured - pid->prev_error) / dt;   // 测量值变化率
        float d_raw = pid->kd * deriv;                      // 原始微分输出
        // 一阶低通滤波：d_filtered = d_filtered * (1-α) + d_raw * α
        pid->d_filtered = pid->d_filtered * 0.7f + d_raw * 0.3f;   // α=0.3
        d_out = pid->d_filtered;
    } else {
        pid->d_filtered = 0.0f;
    }
    pid->prev_error = measured;   // 更新上次测量值

    // 5. 合成输出并限幅
    float output = p_out + i_out + d_out;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}