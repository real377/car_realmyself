/*********************************************************************************************************************
* 文件名称          pid
* 适用平台          MSPM0G3507
* 说明             - 增量式 PID：适合速度闭环
*                  - 位置式 PID：适合角度/位置/循迹闭环
*                  - 所有参数可在运行时动态调整
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

/* ==================== PID 结构体 ==================== */
typedef struct
{
    /* --- 用户可调参数 --- */
    float   kp;                 // 比例系数
    float   ki;                 // 积分系数
    float   kd;                 // 微分系数
    float   target_val;         // 目标值
    float   now;                // 当前测量值

    /* --- 限幅参数 --- */
    float   integral_limit;     // 积分限幅（防止积分饱和）
    float   output_limit;       // 输出限幅（对称，如 7000 表示 ±7000）
    float   dead_zone;          // 死区（误差绝对值小于此值时输出不变）

    /* --- 内部状态（勿手动修改） --- */
    float   ek;                 // 当前误差
    float   ek1;                // 上一次误差
    float   ek2;                // 上上次误差（增量式用）
    float   integral;           // 积分累加（位置式用）
    float   pid_delta;          // 增量式增量
    float   output_val;         // 当前输出值
    float   output_val_last;    // 上一次输出（低通滤波用）

    /* --- 辅助标志 --- */
    uint8   mode;               // PID 模式（见 pid_mode_enum）
    uint8   first_calc;         // 首次计算标志

} PID_TypeDef;

/* ==================== 函数声明 ==================== */

// PID 初始化（设置模式、参数、限幅）
void pid_init(PID_TypeDef *pid, uint8 mode,
              float kp, float ki, float kd,
              float target, float out_limit,
              float integral_limit, float dead_zone);

// PID 计算（每个控制周期调用一次）
float pid_calc(PID_TypeDef *pid, float now);

// PID 参数在线调整
void pid_set_param(PID_TypeDef *pid, float kp, float ki, float kd);
void pid_set_target(PID_TypeDef *pid, float target);
void pid_set_limit(PID_TypeDef *pid, float out_limit, float integral_limit);

// PID 复位（清零积分和历史误差）
void pid_reset(PID_TypeDef *pid);

#endif