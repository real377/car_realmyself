#ifndef __TABLE_H
#define __TABLE_H


#include "zf_common_headfile.h"
#include "zf_common_typedef.h"
#include "zf_common_function.h"


// ==================== 高度表格参数 ====================
#define P_TABLE_MIN     2.7f
#define P_TABLE_MAX     14.5f
#define P_TABLE_STEP    0.1f
#define P_TABLE_SIZE    119     // (14.5 - 2.7) / 0.1 + 1 = 119

// ==================== PWM1 高度控制表（对应原表格角度索引0，即-20度） ====================
extern const int16 pwm1_height_table[P_TABLE_SIZE];

// ==================== PWM2 高度控制表（对应原表格角度索引0，即-20度） ====================
extern const int16 pwm2_height_table[P_TABLE_SIZE];

void servo_control_height(float p, int16* pwm1, int16* pwm2);


#endif