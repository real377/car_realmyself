/*********************************************************************************************************************
* 文件名称          encoder
* 适用平台          MSPM0G3507
* 说明             基于 EXTI 的编码器速度检测（2 倍频解码）
********************************************************************************************************************/

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/* ==================== 编码器硬件引脚定义 ==================== */
#define ENCODER_L_PIN1      B5      // 左编码器 A 相E2A
#define ENCODER_L_PIN2      A10     // 左编码器 B 相E2B
#define ENCODER_R_PIN1      A9      // 右编码器 A 相E1B
#define ENCODER_R_PIN2      B15     // 右编码器 B 相E1A

/* ==================== 编码器参数宏（可按实际调整） ==================== */
#define ENCODER_PPR         170     // 编码器线数（每圈脉冲数，1倍频）
#define ENCODER_MULT         2      // 倍频数（2倍频）
#define ENCODER_PULSE_PER_REV  (ENCODER_PPR * ENCODER_MULT)  // 每圈脉冲数

#define WHEEL_DIAMETER_MM    45     // 轮子直径（mm）
#define WHEEL_PERIMETER_MM   (WHEEL_DIAMETER_MM * 3.14159f)  // 轮子周长（mm）
#define MM_PER_PULSE         (WHEEL_PERIMETER_MM / ENCODER_PULSE_PER_REV) // 每脉冲距离

/* ==================== 全局变量（供速度环 & 里程计算使用） ==================== */
extern volatile int16_t  g_encL_count;       // 左编码器脉冲累计（在定时中断中读取后清零）
extern volatile int16_t  g_encR_count;       // 右编码器脉冲累计
extern volatile int32_t  g_encL_pulse_sum;   // 左编码器总脉冲（里程累计，不清零）
extern volatile int32_t  g_encR_pulse_sum;   // 右编码器总脉冲

extern int16_t  g_speedL_now;   // 左轮当前速度（脉冲/控制周期）
extern int16_t  g_speedR_now;   // 右轮当前速度
extern float    g_distance_mm;  // 累计行驶距离（mm）

/* ==================== 函数声明 ==================== */
void encoder_init(void);
void encoder_clear(void);                    // 清零脉冲计数（在定时中断中调用）
void encoder_speed_calc(uint16 period_ms);   // 速度计算（在定时中断中调用，period_ms 为控制周期）

#endif