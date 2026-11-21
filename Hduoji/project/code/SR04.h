#ifndef __SR04_H
#define __SR04_H

#include "zf_common_headfile.h"

// ============ 引脚定义 ============
#define SR04_TRIG_PIN      B21          // 触发信号引脚 (输出)
#define SR04_ECHO_PIN      A7          // 回波信号引脚 (输入，EXTI中断)

// ============ 常量定义 ============
#define SR04_MAX_DISTANCE  400         // 最大测量距离 400cm
#define SR04_MIN_DISTANCE  2           // 最小测量距离 2cm
#define SR04_TIMEOUT_MS    30          // 超时时间 (ms) 约对应5米距离

// ============ 全局变量声明 ============
extern volatile uint8_t  sr04_measure_done;    // 测量完成标志
extern volatile uint32_t sr04_high_time_us;    // 高电平时间(微秒)
extern int Distance;                            // 距离值 (cm)

// ============ 函数声明 ============
void Sr04_All_Init(void);                      // SR04 全部初始化
int Get_SR04_Distance(void);                   // 获取距离值 (cm)
void sr04_echo_exti_callback(uint32 event, void *ptr);  // EXTI 回调函数
void LightSensor_Init(void);
int LightSensor_Get(void);

#endif