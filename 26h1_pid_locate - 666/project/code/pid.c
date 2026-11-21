/**
 * @file    pid.c
 * @brief   PID 控制器实现
 */

#include "pid.h"

/**
 * @brief   初始化 PID 控制器
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit, float o_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->integral_limit = (i_limit > 0.0f) ? i_limit : 0.0f;
    pid->output_limit   = (o_limit > 0.0f) ? o_limit : 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief   执行 PID 计算
 * @note    微分项采用“微分先行”思想，对测量值变化率进行微分，可避免设定值突变导致的冲击。
 *          如需经典微分（误差的微分），可自行修改。
 */
float pid_update(pid_t *pid, float setpoint, float measured, float dt)
{
    /* 1. 计算误差 */
    float error = setpoint - measured;

    /* 2. 比例项 */
    float p_out = pid->kp * error;

    /* 3. 积分项（带抗饱和） */
    pid->integral += error * dt;
    /* 对积分进行对称限制 */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->ki * pid->integral;

    /* 4. 微分项 —— 采用测量值微分（避免设定值突变造成超调） */
    float d_out = 0.0f;
    if (dt > 1e-6f) {   // 防止除零
        /* 测量值变化率：-(measured - prev_measured)/dt ，注意符号 */
        float deriv = -(measured - pid->prev_error) / dt;  // 这里复用了 prev_error 存储上次测量值
        d_out = pid->kd * deriv;
    }
    /* 更新上一次测量值（用于下次微分） */
    pid->prev_error = measured;

    /* 5. 合成输出并限幅 */
    float output = p_out + i_out + d_out;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}