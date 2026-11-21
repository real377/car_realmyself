/*********************************************************************************************************************
* 文件名称          gray
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动（整数运算，低运算量）
*                  - 加权偏差计算（整数乘除，无浮点）
*                  - 转向差速输出，限幅 ±500
*                  - 简易状态识别（全黑 / 边缘 / 丢线 / 正常）
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "gray.h"

/* ==================== 全局数据定义 ==================== */
GrayData_TypeDef g_gray = {0};

/* ==================== 传感器位置权重（中间为 0） ==================== */
static const int8 gray_weight[7] = {-10, -6, -3, 0, 3, 6, 10};

/* ==================== GPIO 引脚数组 ==================== */
static const gpio_pin_enum gray_pins[7] =
{
    GRAY_L1_PIN,   // [0] L3
    GRAY_L2_PIN,   // [1] L2
    GRAY_L3_PIN,   // [2] L1
    GRAY_MID_PIN,  // [3] MID
    GRAY_R1_PIN,   // [4] R1
    GRAY_R2_PIN,   // [5] R2
    GRAY_R3_PIN    // [6] R3
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器 GPIO 初始化
// 备注信息     7 路均配置为输入 + 下拉电阻
//-------------------------------------------------------------------------------------------------------------------
void gray_init(void)
{
    uint8 i;
    for(i = 0; i < 7; i++)
    {
        gpio_init(gray_pins[i], GPI, GPIO_LOW, GPI_PULL_DOWN);
    }
    memset(&g_gray, 0, sizeof(GrayData_TypeDef));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新灰度传感器数据（每个控制周期调用一次，如 5~10ms）
// 备注信息     1. 读取原始电平 -> 检测线标志
//              2. 计算加权偏差（整数运算）
//              3. 计算转向差速（限幅 ±500）
//              4. 识别基本状态
//-------------------------------------------------------------------------------------------------------------------
void gray_update(void)
{
    uint8 i;
    int32 sum_weighted = 0;
    int32 sum_active   = 0;
    static int16 last_deviation = 0;    // 丢线时保持上一次偏差

    // ---------- 第1步：读取7路传感器 ----------
    g_gray.active_count = 0;
    for(i = 0; i < 7; i++)
    {
        g_gray.raw[i] = gpio_get_level(gray_pins[i]);
        if(g_gray.raw[i] == GRAY_LINE_LEVEL)
        {
            g_gray.active[i] = 1;
            g_gray.active_count++;
        }
        else
        {
            g_gray.active[i] = 0;
        }
    }

    // ---------- 第2步：加权偏差计算（纯整数） ----------
    if(g_gray.active_count > 0)
    {
        for(i = 0; i < 7; i++)
        {
            if(g_gray.active[i])
            {
                sum_weighted += gray_weight[i];
                sum_active   += 1;
            }
        }
        // 偏差 = 加权平均 × SCALE，并限幅
        g_gray.deviation = (int16)((sum_weighted * GRAY_DEVIATION_SCALE) / sum_active);
        if(g_gray.deviation >  GRAY_MAX_DEVIATION) g_gray.deviation =  GRAY_MAX_DEVIATION;
        if(g_gray.deviation < -GRAY_MAX_DEVIATION) g_gray.deviation = -GRAY_MAX_DEVIATION;
        last_deviation = g_gray.deviation;
    }
    else
    {
        // 完全丢线：保持上一次偏差
//        g_gray.deviation = last_deviation;
    }

    // ---------- 第3步：转向差速输出（限幅 ±500） ----------
    // 公式：turn_diff = deviation * MAX_TURN_DIFF / MAX_DEVIATION
    g_gray.turn_diff = (int16)g_gray.deviation ;
    if(g_gray.turn_diff >  GRAY_MAX_TURN_DIFF) g_gray.turn_diff =  GRAY_MAX_TURN_DIFF;
    if(g_gray.turn_diff < -GRAY_MAX_TURN_DIFF) g_gray.turn_diff = -GRAY_MAX_TURN_DIFF;

    // ---------- 第4步：简单状态识别 ----------
    if(g_gray.active_count >= 7)
    {
        g_gray.state = GRAY_STATE_ALL_BLACK;        // 全黑
    }
    else if( (g_gray.active[0] || g_gray.active[1] || g_gray.active[2]) &&   // 左侧有信号
             !g_gray.active[3] &&                                           // 中间无信号
             !(g_gray.active[4] || g_gray.active[5] || g_gray.active[6]) )   // 右侧无信号
    {
        g_gray.state = GRAY_STATE_LEFT_EDGE;        // 左边缘
    }
    else if( !(g_gray.active[0] || g_gray.active[1] || g_gray.active[2]) &&   // 左侧无信号
             !g_gray.active[3] &&                                           // 中间无信号
             (g_gray.active[4] || g_gray.active[5] || g_gray.active[6]) )   // 右侧有信号
    {
        g_gray.state = GRAY_STATE_RIGHT_EDGE;       // 右边缘
    }
		
		
		
		
    else if(g_gray.active_count == 0)
    {
        g_gray.state = GRAY_STATE_LOST;             // 丢线
    }
    else
    {
        g_gray.state = GRAY_STATE_NORMAL;           // 正常循迹
    }
		
		
}

//-------------------------------------------------------------------------------------------------------------------
// 便捷接口
//-------------------------------------------------------------------------------------------------------------------
int16 gray_get_deviation(void)
{
    return g_gray.deviation;
}

int16 gray_get_turn_diff(void)
{
    return g_gray.turn_diff;
}

uint8 gray_get_state(void)
{
    return g_gray.state;
}

uint8 gray_is_lost(void)
{
    return (g_gray.state == GRAY_STATE_LOST);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 调试显示
// 备注信息     在 OLED 第 1~4 行显示传感器状态、偏差、差速
//             需包含 "OLED.h" 并调用 OLED_Init()
//-------------------------------------------------------------------------------------------------------------------
void gray_debug_oled(void)
{
    char buf[17] = {0};
    uint8 i;
    // 第1行：7 路传感器状态（# 表示检测到线）
    for(i = 0; i < 7; i++)
    {
        buf[i * 2]     = g_gray.active[i] ? '#' : '.';
        buf[i * 2 + 1] = ' ';
    }
    buf[14] = '\0';
    OLED_ShowString(1, 1, buf);

    // 第2行：偏差值
    OLED_ShowSignedNum(2, 1, g_gray.deviation, 5);

    // 第3行：差速输出
    OLED_ShowSignedNum(3, 1, g_gray.turn_diff, 4);

    // 第4行：状态
    const char *state_str[] = {"NORM","LOST","BLK","L_EDG","R_EDG"};
    OLED_ShowString(4, 1, (char*)state_str[g_gray.state]);
}