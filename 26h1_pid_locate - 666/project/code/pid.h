/**
 * @file    pid.h
 * @brief   通用 PID 控制器接口
 * @note    可用于位置环、速度环等，支持积分分离/抗饱和与输出限幅
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/*  PID 控制器状态结构体  */
typedef struct {
    float kp;                  // 比例系数
    float ki;                  // 积分系数
    float kd;                  // 微分系数
    float integral;            // 积分累加量
    float integral_limit;      // 积分上限（对称限制，正负均适用）
    float output_limit;        // 输出限幅（对称限制）
    float prev_error;          // 上一次误差（用于微分）
} pid_t;

/**
 * @brief   初始化 PID 控制器
 * @param   pid     控制器句柄
 * @param   kp      比例系数
 * @param   ki      积分系数
 * @param   kd      微分系数
 * @param   i_limit 积分饱和上限（>0）
 * @param   o_limit 输出限幅（>0）
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit, float o_limit);

/**
 * @brief   执行一次 PID 计算
 * @param   pid        控制器句柄
 * @param   setpoint   设定值
 * @param   measured   测量值
 * @param   dt         控制周期（单位：秒），必须 >0
 * @return  计算得到的控制输出（已限幅）
 */
float pid_update(pid_t *pid, float setpoint, float measured, float dt);

#endif /* __PID_H__ */