/*********************************************************************************************************************
* 文件名称          pid
* 适用平台          MSPM0G3507
********************************************************************************************************************/

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 初始化
// 参数说明     mode            0=增量式, 1=位置式
//             kp/ki/kd        PID 参数
//             target          目标值
//             out_limit       输出限幅（对称，如 7000 则输出范围 ±7000）
//             integral_limit  积分限幅
//             dead_zone       死区
//-------------------------------------------------------------------------------------------------------------------
void pid_init(PID_TypeDef *pid, uint8 mode,
              float kp, float ki, float kd,
              float target, float out_limit,
              float integral_limit, float dead_zone)
{
    pid->kp              = kp;
    pid->ki              = ki;
    pid->kd              = kd;
    pid->target_val      = target;
    pid->now             = 0;
    pid->output_limit    = out_limit;
    pid->integral_limit  = integral_limit;
    pid->dead_zone       = dead_zone;
    pid->mode            = mode;

    pid->ek              = 0;
    pid->ek1             = 0;
    pid->ek2             = 0;
    pid->integral        = 0;
    pid->pid_delta       = 0;
    pid->output_val      = 0;
    pid->output_val_last = 0;
    pid->first_calc      = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 计算
// 参数说明     now     当前测量值
// 返回参数     输出值
// 备注信息     增量式：Δu = Kp*(ek-ek1) + Ki*ek + Kd*(ek-2*ek1+ek2)
//             位置式：u  = Kp*ek + Ki*Σek + Kd*(ek-ek1)
//-------------------------------------------------------------------------------------------------------------------
float pid_calc(PID_TypeDef *pid, float now)
{
    // 更新误差
    pid->now = now;
    pid->ek2 = pid->ek1;
    pid->ek1 = pid->ek;
    pid->ek  = pid->target_val - pid->now;

    // --- 死区处理 ---
    if(fabs(pid->ek) < pid->dead_zone)
    {
        pid->ek = 0;
    }

    // --- 根据模式计算 ---
    if(pid->mode == PID_MODE_INCREMENTAL)
    {
        // 增量式 PID
        pid->pid_delta = pid->kp * (pid->ek - pid->ek1)
                       + pid->ki * pid->ek
                       + pid->kd * (pid->ek - 2.0f * pid->ek1 + pid->ek2);

        pid->output_val += pid->pid_delta;
    }
    else
    {
        // 位置式 PID
        pid->integral += pid->ek;

        // 积分限幅
        if(pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

        pid->output_val = pid->kp * pid->ek
                        + pid->ki * pid->integral
                        + pid->kd * (pid->ek - pid->ek1);
    }

    // --- 输出限幅 ---
    if(pid->output_val >  pid->output_limit) pid->output_val =  pid->output_limit;
    if(pid->output_val < -pid->output_limit) pid->output_val = -pid->output_limit;

    // --- 低通滤波平滑 ---
    pid->output_val = 0.8f * pid->output_val + 0.2f * pid->output_val_last;
    pid->output_val_last = pid->output_val;

    pid->first_calc = 0;
    return pid->output_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 在线调整 PID 参数
//-------------------------------------------------------------------------------------------------------------------
void pid_set_param(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_target(PID_TypeDef *pid, float target)
{
    pid->target_val = target;
}

void pid_set_limit(PID_TypeDef *pid, float out_limit, float integral_limit)
{
    pid->output_limit   = out_limit;
    pid->integral_limit = integral_limit;
}

//-------------------------------------------------------------------------------------------------------------------
// PID 复位
//-------------------------------------------------------------------------------------------------------------------
void pid_reset(PID_TypeDef *pid)
{
    pid->ek         = 0;
    pid->ek1        = 0;
    pid->ek2        = 0;
    pid->integral   = 0;
    pid->pid_delta  = 0;
    pid->output_val = 0;
    pid->output_val_last = 0;
    pid->first_calc = 1;
}