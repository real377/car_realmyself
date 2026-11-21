/**
 * @file    duoji.h
 * @brief   舵机控制 — 开环四模式舵机控制器
 *          MODE_AUTO  ：自动循环正弦往复
 *          MODE_CAMERA：摄像头触发 — 线性步进偏移 + 快速回中
 *          MODE_PID   ：摄像头触发 — 伪PID指数衰减偏移 + 快速回中
 *          MODE_SWEEP ：一键扫描 — 中值上下可配置升降
 *          MODE_DYNAMIC：摄像头动态触发 — 方向指令触发一次去回，参数可实时调次数与幅度
 */

#ifndef __DUOJI_H
#define __DUOJI_H

#include "zf_common_headfile.h"

/* ====== 硬件定义 ====== */
#define SERVO_1                 (PWM_TIM_A0_CH2_A7)
#define SERVO_2                 (PWM_TIM_G8_CH0_B21)
#define SERVO_FREQ              (300)

#define SERVO1_MID              2000
#define SERVO2_MID              4900

/* ====== 运动模式 ====== */
#define DUOJI_MODE_IDLE         0   /* 停止，保持中位 */
#define DUOJI_MODE_AUTO         1   /* 自动循环正弦往复 */
#define DUOJI_MODE_CAMERA       2   /* 摄像头触发：线性步进偏移 + 回中 */
#define DUOJI_MODE_PID          3   /* 摄像头触发：伪PID指数平滑偏移 + 回中 */
#define DUOJI_MODE_SWEEP        4   /* 一键扫描：中值上下可配置升降 */
#define DUOJI_MODE_DYNAMIC      5   /* 摄像头动态触发一次去回 */

/* ====== 摄像头指令 ====== */
#define DUOJI_CAMERA_STOP       0   /* 停止 → 回中位 */
#define DUOJI_CAMERA_UP         1   /* 左转 → 向上偏移 */
#define DUOJI_CAMERA_DOWN       2   /* 右转 → 向下偏移 */

/* ====== 配置参数 ====== */
typedef struct {
    float amplitude;        /* AUTO 模式峰值 PWM 偏移量（如 200.0f） */
    float frequency;        /* AUTO 模式振荡频率 Hz（如 2.0f） */
    int16 camera_offset;    /* CAMERA/PID 模式单次偏移幅度（如 150） */
    int16 camera_step;      /* CAMERA 偏移步长（如 12）              */
    int16 camera_step_back; /* CAMERA 回中步长（如 25）              */
    float pid_kp;           /* PID 模式比例系数（如 0.20f）         */
    float pid_kp_back;      /* PID 模式回中比例系数（如 0.35f）      */
    int16 pid_max_step;     /* PID 最大步长上限（如 12）             */
    int16 pid_max_step_back;/* PID 回中最大步长上限（如 25）         */
    int16 sweep_amplitude;  /* SWEEP 升降幅度 PWM（如 150）         */
    int16 sweep_step;       /* SWEEP 去程步长（离开中位，如 10）     */
    int16 sweep_step_back;  /* SWEEP 回程步长（回到中位，如 20）     */
    uint8_t sweep_cnt_up;   /* SWEEP 中值以上升降次数（0=不升）      */
    uint8_t sweep_cnt_down; /* SWEEP 中值以下升降次数（0=不降）      */
} duoji_cfg_t;

/* ====== 接口函数 ====== */
void  duoji_init(void);
void  duoji_set_mode(uint8_t mode);
void  duoji_set_camera_cmd(uint8_t cmd);
void  duoji_set_sweep_amp(int16 amp);          /* DYNAMIC 动态调幅 */
void  duoji_set_sweep_step(int16 out, int16 back); /* DYNAMIC 动态调步长 */
void  duoji_apply_config(const duoji_cfg_t *cfg);
void  duoji_update(uint32_t dt_ms);
int16 duoji_get_pwm_offset(void);


#endif /* __DUOJI_H */
