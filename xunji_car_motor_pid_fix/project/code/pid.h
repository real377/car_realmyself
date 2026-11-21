/*********************************************************************************************************************
* 文件名称          pid
* 适用平台          MSPM0G3507
* 说明             定点整数 PID 控制器（Q10 格式，无浮点运算）
*                  - 增量式 PID：适合速度闭环
*                  - 位置式 PID：适合角度/位置/循迹闭环
*                  - 所有参数可在运行时动态调整
*                  - 所有系数、限幅值均采用 Q10 定点格式（×1024）
********************************************************************************************************************/

#ifndef _PID_H_
#define _PID_H_

#include "zf_common_typedef.h"

/* ==================== PID 模式选择 ==================== */
typedef enum
{
    PID_MODE_INCREMENTAL = 0,   // 增量式
    PID_MODE_POSITIONAL  = 1    // 位置式
} pid_mode_enum;

/* ==================== PID 结构体（所有变量均 int32_t） ==================== */
typedef struct
{
    /* --- 用户可调参数（Q10 定点格式，真实值 = 参数值 / 1024） --- */
    int32_t kp;                 // 比例系数 × 1024
    int32_t ki;                 // 积分系数 × 1024
    int32_t kd;                 // 微分系数 × 1024
    int32_t target_val;         // 目标值（实际量程，如灰度偏差目标为 0）

    /* --- 限幅参数（实际量程，无需缩放） --- */
    int32_t integral_limit;     // 积分限幅（防止积分饱和）
    int32_t output_limit;       // 输出限幅（对称，如 7000 表示 ±7000）
    int32_t dead_zone;          // 死区（误差绝对值小于此值时输出不变）

    /* --- 内部状态（勿手动修改） --- */
    int32_t now;                // 当前测量值（实际量程）
    int32_t ek;                 // 当前误差
    int32_t ek1;                // 上一次误差
    int32_t ek2;                // 上上次误差（增量式用）
    int32_t integral;           // 积分累加（位置式用，实际量程，未缩放）
    int32_t pid_delta;          // 增量式增量（实际量程）
    int32_t output_val;         // 当前输出值（实际量程，可直接用于 PWM）
    int32_t output_val_last;    // 上一次输出（低通滤波用）

    /* --- 辅助标志 --- */
    uint8_t mode;               // PID 模式（见 pid_mode_enum）
    uint8_t first_calc;         // 首次计算标志

} PID_TypeDef;

/* ==================== 函数声明 ==================== */

// PID 初始化（所有系数、限幅均使用 Q10 定点值）
void pid_init(PID_TypeDef *pid, uint8_t mode,
              int32_t kp, int32_t ki, int32_t kd,
              int32_t target, int32_t out_limit,
              int32_t integral_limit, int32_t dead_zone);

// PID 计算（每个控制周期调用一次，传入实际测量值，返回实际输出值）
int32_t pid_calc(PID_TypeDef *pid, int32_t now);

// PID 参数在线调整（参数均为 Q10 定点值）
void pid_set_param(PID_TypeDef *pid, int32_t kp, int32_t ki, int32_t kd);
void pid_set_target(PID_TypeDef *pid, int32_t target);
void pid_set_limit(PID_TypeDef *pid, int32_t out_limit, int32_t integral_limit);

// PID 复位（清零积分和历史误差）
void pid_reset(PID_TypeDef *pid);

#endif