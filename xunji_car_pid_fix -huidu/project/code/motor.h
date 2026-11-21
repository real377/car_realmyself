/*********************************************************************************************************************
* 文件名称          motor
* 公司名称          成都逐飞科技
* 版本信息          参见库版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"


/* ==================== 电机硬件引脚定义 ==================== */
#define PWM_L               PWM_TIM_A0_CH3_A28   // 左电机 PWMB
#define PWM_R               PWM_TIM_A1_CH1_B3    // 右电机 PWMA
#define DIR_L1              A31                  // 左电机方向1BIN2
#define DIR_L2              B4                   // 左电机方向2
#define DIR_R1              B2                   // 右电机方向1
#define DIR_R2              B16                  // 右电机方向2AIN1
#define MOTOR_PWM_FREQ      17000                // PWM 频率（Hz）
/* ==================== 电机初始化 ==================== */



/* ==================== 电机初始化 ==================== */
void motor_init(void);

/* ==================== 电机控制函数 ==================== */
void motor_set_left(int32 duty);                 // 左电机控制，正值为正转，负值为反转
void motor_set_right(int32 duty);                // 右电机控制，正值为正转，负值为反转
void motor_set_speed(int32 dutyL, int32 dutyR);  // 同时设置两电机
void motor_stop(void);                           // 电机停止（刹车）

#endif