/*********************************************************************************************************************
* 文件名称          test
* 适用平台          MSPM0G3507
* 说明             分级测试函数，从底层到高层逐步验证
********************************************************************************************************************/

#ifndef _TEST_H_
#define _TEST_H_

#include "zf_common_typedef.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

/* ==================== 全局 PID 对象声明 ==================== */
extern PID_TypeDef g_pidSpeedL;      // 左轮速度环 PID
extern PID_TypeDef g_pidSpeedR;      // 右轮速度环 PID

/* ==================== 测试函数声明 ==================== */

// 测试 1：电机基础测试（正反转、停止）
// 用途：验证 PWM 输出和方向控制是否正常
void test_motor_basic(void);

// 测试 2：编码器测试（手动推车，观察脉冲计数）
// 用途：验证编码器接线和 EXTI 中断是否正常
// 需在 OLED 上显示 g_encL_count 和 g_encR_count
void test_encoder_read(void);

// 测试 3：PID 速度闭环测试（定速行驶）
// 用途：验证速度闭环是否稳定，调节 Kp/Ki
// 参数：target_speed 目标速度（脉冲/控制周期），duration_ms 测试时长
void test_pid_speed(int16 target_speed, uint32 duration_ms);

// 测试 4：综合行驶测试（走直线 + 定距停车）
// 用途：验证编码器里程 + PID 速度控制的综合效果
// 参数：target_distance_mm 目标距离（mm），target_speed 目标速度
void test_distance_run(float target_distance_mm, int16 target_speed);

// 速度控制回调（在 PIT 中断中调用，供测试 3/4 使用）
void test_speed_control_callback(uint32 state, void *ptr);

#endif