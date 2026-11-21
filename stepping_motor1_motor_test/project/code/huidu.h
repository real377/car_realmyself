/*********************************************************************************************************************
* 文件名称          huidu
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动（正序排列）
*                  - 偏差输出：0 表示线居中，正值线偏右，负值线偏左
*                  - 拐角/路口状态识别，供上层决策
********************************************************************************************************************/
#ifndef _HUIDU_H_
#define _HUIDU_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

// ---------- 引脚定义（保持用户原有命名） ----------
#define HUIDU_L1_PIN        B6
#define HUIDU_L2_PIN        B8
#define HUIDU_L3_PIN        B9
#define HUIDU_MID_PIN       B7
#define HUIDU_R1_PIN        B26
#define HUIDU_R2_PIN        A12
#define HUIDU_R3_PIN        A13

// ---------- 电平有效定义 ----------
#define HUIDU_LINE_LEVEL    1       // 检测到引导线时 GPIO 为高电平

// 在 huidu.h 中增加以下宏定义
#define HUIDU_DEVIATION_MAX     500    // 偏差输出最大值（对称）
#define HUIDU_FILTER_ALPHA      0.7f    // 低通滤波系数（0~1，越小滤波越强）

// ---------- 状态枚举 ----------
typedef enum
{
    HUIDU_STATE_NORMAL      = 0,    // 正常循迹
    HUIDU_STATE_LOST        = 1,    // 完全丢线（7 路全无）
    HUIDU_STATE_CROSS       = 2,    // 十字路口（7 路全检测到）
    HUIDU_STATE_T_LEFT      = 4,    // 左岔路（左侧全亮 + 中间 + 右侧部分亮）
    HUIDU_STATE_T_RIGHT     = 3,    // 右岔路（右侧全亮 + 中间 + 左侧部分亮）
    HUIDU_STATE_SHARP_LEFT  = 6,    // 急左弯（仅左侧有探测）
    HUIDU_STATE_SHARP_RIGHT = 5,    // 急右弯（仅右侧有探测）
    HUIDU_STATE_ALL_BLACK   = 7     // 全黑（预留）
} huidu_state_t;

// ---------- 全局数据结构 ----------
typedef struct
{
    uint8   raw[7];          // 原始电平
    uint8   active[7];       // 稳定状态（1=检测到线）
    int16   deviation;       // 加权偏差（用于循迹 PID）
    uint8   state;           // 当前状态
    uint8   active_count;    // 检测到线的数量
} huidu_data_t;

extern huidu_data_t g_huidu;

// ---------- 函数声明 ----------
void huidu_init(void);
void huidu_update(void);                            // 每个控制周期调用一次
int16 huidu_get_deviation(void);
uint8 huidu_get_state(void);
uint8 huidu_is_lost(void);
uint8 huidu_is_cross(void);
void huidu_debug_oled_4line(int16 speedL, int16 speedR);  // 四行 OLED 显示
#endif