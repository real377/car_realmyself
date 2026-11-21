/*********************************************************************************************************************
* 文件名称          pid
* 适用平台          MSPM0G3507
* 说明             定点整数 PID 控制器（Q10 格式）
*                  所有运算均为整数，无浮点，适合无 FPU 的 MCU
********************************************************************************************************************/

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID 初始化（全部使用 Q10 定点值）
// 参数说明     mode            0=增量式, 1=位置式
//             kp/ki/kd        PID 参数（×1024 的定点值）
//             target          目标值（实际量程，如 0）
//             out_limit       输出限幅（实际量程，如 7000 表示 ±7000）
//             integral_limit  积分限幅（实际量程）
//             dead_zone       死区（实际量程）
//-------------------------------------------------------------------------------------------------------------------
void pid_init(PID_TypeDef *pid, uint8_t mode,
              int32_t kp, int32_t ki, int32_t kd,
              int32_t target, int32_t out_limit,
              int32_t integral_limit, int32_t dead_zone)
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
// 函数简介     PID 计算（纯整数，Q10 定点运算）
// 参数说明     now     当前测量值（实际量程）
// 返回参数     输出值（实际量程）
// 备注信息     增量式：Δu = (Kp*(ek-ek1) + Ki*ek + Kd*(ek-2*ek1+ek2)) / 1024
//             位置式：u  = (Kp*ek + Ki*Σek + Kd*(ek-ek1)) / 1024
//-------------------------------------------------------------------------------------------------------------------
int32_t pid_calc(PID_TypeDef *pid, int32_t now)
{
    // 更新误差
    pid->now = now;
    pid->ek2 = pid->ek1;
    pid->ek1 = pid->ek;
    pid->ek  = pid->target_val - pid->now;

    // --- 死区处理 ---
    // 注意：此处直接使用 abs(ek) 比较，ek 为实际量程
    if ((pid->ek >= 0 ? pid->ek : -pid->ek) < pid->dead_zone)
    {
        pid->ek = 0;
    }

    // --- 根据模式计算 ---
    if (pid->mode == PID_MODE_INCREMENTAL)
    {
        // 增量式 PID：三项乘积均需右移 10 位还原为实际量程
        int32_t p_delta = (pid->kp * (pid->ek - pid->ek1)) >> 10;
        int32_t i_delta = (pid->ki * pid->ek) >> 10;
        int32_t d_delta = (pid->kd * (pid->ek - 2 * pid->ek1 + pid->ek2)) >> 10;

        pid->pid_delta = p_delta + i_delta + d_delta;
        pid->output_val += pid->pid_delta;
    }
    else
    {
        // 位置式 PID
        // 积分累加（使用实际量程，不受 Q10 影响）
        pid->integral += pid->ek;

        // 积分限幅
        if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

        // 计算各项（右移 10 位还原）
        int32_t p_term = (pid->kp * pid->ek) >> 10;
        int32_t i_term = (pid->ki * pid->integral) >> 10;   // integral 已是实际量程，无需缩放
        int32_t d_term = (pid->kd * (pid->ek - pid->ek1)) >> 10;

        pid->output_val = p_term + i_term + d_term;
    }

    // --- 输出限幅 ---
    if (pid->output_val >  pid->output_limit) pid->output_val =  pid->output_limit;
    if (pid->output_val < -pid->output_limit) pid->output_val = -pid->output_limit;

    // --- 低通滤波平滑（用 α=0.75 的移位近似） ---
    // output_val = (output_val * 3 + output_val_last) / 4
    pid->output_val = (pid->output_val * 3 + pid->output_val_last) >> 2;
    pid->output_val_last = pid->output_val;

    pid->first_calc = 0;
    return pid->output_val;
}

//-------------------------------------------------------------------------------------------------------------------
// 在线调整 PID 参数（参数均为 Q10 定点值）
//-------------------------------------------------------------------------------------------------------------------
void pid_set_param(PID_TypeDef *pid, int32_t kp, int32_t ki, int32_t kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_target(PID_TypeDef *pid, int32_t target)
{
    pid->target_val = target;
}

void pid_set_limit(PID_TypeDef *pid, int32_t out_limit, int32_t integral_limit)
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