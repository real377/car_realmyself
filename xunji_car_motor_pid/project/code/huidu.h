/*********************************************************************************************************************
* 文件名称          huidu
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动
*                  - 传感器状态读取
*                  - 加权偏差计算（用于循迹 PID 转向修正）
*                  - 路口/特殊状态识别
* 传感器布局        L3  L2  L1  MID  R1  R2  R3
*                  B9  B8  B6  B7  B26 A12 A13
********************************************************************************************************************/

#ifndef _HUIDU_H_
#define _HUIDU_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/* ==================== 传感器引脚定义 ==================== */
#define HUIDU_L1_PIN        B6
#define HUIDU_L2_PIN        B8
#define HUIDU_L3_PIN        B9
#define HUIDU_MID_PIN       B7
#define HUIDU_R1_PIN        B26
#define HUIDU_R2_PIN        A12
#define HUIDU_R3_PIN        A13

/* ==================== 检测电平配置 ==================== */
// 检测到引导线时的 GPIO 电平。通常灰度传感器检测到深色线输出低电平。
// 如果实际传感器相反，改为 1 即可。
#define HUIDU_LINE_LEVEL    0

/* ==================== 偏差输出范围 ==================== */
#define HUIDU_DEVIATION_MAX     3000    // 最大偏差值（线在最外侧）
#define HUIDU_DEVIATION_MIN    -3000    // 最小偏差值

/* ==================== 传感器状态枚举 ==================== */
typedef enum
{
    HUIDU_STATE_NORMAL      = 0,    // 正常循迹中（线在传感器视野内）
    HUIDU_STATE_LOST        = 1,    // 丢线（所有传感器都没检测到）
    HUIDU_STATE_CROSS       = 2,    // 十字路口（全部检测到线）
    HUIDU_STATE_T_LEFT      = 3,    // T字路口（左侧有岔路）
    HUIDU_STATE_T_RIGHT     = 4,    // T字路口（右侧有岔路）
    HUIDU_STATE_LEFT_TURN   = 5,    // 左直角弯（仅左侧传感器检测到）
    HUIDU_STATE_RIGHT_TURN  = 6,    // 右直角弯（仅右侧传感器检测到）
    HUIDU_STATE_ALL_BLACK   = 7,    // 全黑（可能是路口或标记）
} huidu_state_enum;

/* ==================== 传感器原始数据 ==================== */
typedef struct
{
    uint8   raw[7];          // 7 路原始电平值（0 或 1）
    uint8   active[7];       // 7 路"检测到线"标志（1=有线）
    uint8   active_count;    // 检测到线的传感器数量
    int16   deviation;       // 加权偏差（用于循迹 PID）
    uint8   state;           // 当前状态（见 huidu_state_enum）
} HuiduData_TypeDef;

/* ==================== 全局数据 ==================== */
extern HuiduData_TypeDef g_huidu;

/* ==================== 函数声明 ==================== */

// 初始化 7 路灰度传感器 GPIO
void huidu_init(void);

// 读取所有传感器，更新 g_huidu 结构体（每个控制周期调用一次）
void huidu_update(void);

// 获取偏差值（便捷接口）
int16 huidu_get_deviation(void);

// 获取当前循迹状态
uint8 huidu_get_state(void);

// 判断是否丢线
uint8 huidu_is_lost(void);

// 判断是否检测到路口
uint8 huidu_is_cross(void);

// 调试：在 OLED 上显示传感器状态
void huidu_debug_oled(void);

#endif