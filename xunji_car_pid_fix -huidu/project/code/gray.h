/*********************************************************************************************************************
* 文件名称          gray
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动（简化版，无浮点，低运算量）
*                  - 加权偏差计算
*                  - 转向差速输出（限幅 ±500）
*                  - 简易状态识别
********************************************************************************************************************/

#ifndef _GRAY_H_
#define _GRAY_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/* ==================== 传感器引脚定义（与你 huidu 相同，可按需修改） ==================== */
#define GRAY_L1_PIN         B6
#define GRAY_L2_PIN         B8
#define GRAY_L3_PIN         B9
#define GRAY_MID_PIN        B7
#define GRAY_R1_PIN         B26
#define GRAY_R2_PIN         A12
#define GRAY_R3_PIN         A13

/* ==================== 检测电平配置 ==================== */
#define GRAY_LINE_LEVEL     1   // 检测到线时为低电平，若相反请改为 1

/* ==================== 量程规划（可按需调整） ==================== */
#define GRAY_DEVIATION_SCALE    500      // 偏差内部缩放（提高整数精度）
#define GRAY_MAX_DEVIATION      6000      // 理论最大偏差（对应权重3 * SCALE）
#define GRAY_MAX_TURN_DIFF      2000       // 转向差速限幅（左右轮速度差绝对值）

/* ==================== 状态枚举（简化版） ==================== */
typedef enum
{
    GRAY_STATE_NORMAL      = 0,    // 正常循迹
    GRAY_STATE_LOST        = 1,    // 丢线
    GRAY_STATE_ALL_BLACK   = 2,    // 全黑（十字路口或标记）
    GRAY_STATE_LEFT_EDGE   = 3,    // 仅左侧传感器有效（左直角弯等）
    GRAY_STATE_RIGHT_EDGE  = 4,    // 仅右侧传感器有效（右直角弯等）
} gray_state_enum;

/* ==================== 传感器数据结构 ==================== */
typedef struct
{
    uint8   raw[7];          // 原始电平
    uint8   active[7];       // 检测到线标志（1=有线）
    uint8   active_count;    // 检测到线的传感器数量
    int16   deviation;       // 加权偏差，范围 ±GRAY_MAX_DEVIATION
    int16   turn_diff;       // 转向差速输出，范围 ±GRAY_MAX_TURN_DIFF
    uint8   state;           // 当前状态，见 gray_state_enum
} GrayData_TypeDef;

/* ==================== 全局数据 ==================== */
extern GrayData_TypeDef g_gray;

/* ==================== 函数声明 ==================== */
void gray_init(void);                       // 初始化 GPIO
void gray_update(void);                     // 更新所有数据（每个控制周期调用一次）
int16 gray_get_deviation(void);             // 获取偏差
int16 gray_get_turn_diff(void);             // 获取转向差速（已限幅）
uint8 gray_get_state(void);                 // 获取状态
uint8 gray_is_lost(void);                   // 是否丢线
void gray_debug_oled(void);                 // OLED 调试（需链接 OLED 库）

#endif