/*********************************************************************************************************************
* 文件名称          huidu
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动（修正版）
*                  - 物理顺序：L3, L2, L1, MID, R1, R2, R3
*                  - 1ms 防抖 + 加权偏差计算
*                  - 拐角/路口状态识别
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "huidu.h"
#include "oled.h"       // OLED 显示要用

// ================= 全局变量 =================
huidu_data_t g_huidu = {0};

// ---------- 传感器物理位置对应的引脚数组（从左到右：L3 L2 L1 MID R1 R2 R3） ----------
static const gpio_pin_enum pins[7] = {
    HUIDU_L1_PIN,   // [0] L3  最左
    HUIDU_L2_PIN,   // [1] L2
    HUIDU_L3_PIN,   // [2] L1
    HUIDU_MID_PIN,  // [3] MID
    HUIDU_R1_PIN,   // [4] R1
    HUIDU_R2_PIN,   // [5] R2
    HUIDU_R3_PIN    // [6] R3  最右
};

// ---------- 对应索引的权重（用于偏差计算，放大 1000 倍后使用） ----------
// 例：线在 L3 时偏差 = -2000；L2 = -1333；L1 = -667；MID = 0；R1 = 667；R2 = 1333；R3 = 2000
static const int16 pos_weight[7] = {-2000, -1333, -667, 0, 667, 1333, 2000};

//-------------------------------------------------------------------
// 初始化：配置 GPIO 为输入 + 下拉（和原代码一致）
//-------------------------------------------------------------------
void huidu_init(void)
{
    uint8 i;
    for(i=0; i<7; i++)
        gpio_init(pins[i], GPI, GPIO_LOW, GPI_PULL_DOWN);
    memset(&g_huidu, 0, sizeof(huidu_data_t));
}

//-------------------------------------------------------------------
// 传感器更新（每个控制周期调用一次）
// 包含：读取原始值 → 连续防抖 → 计算加权偏差 → 识别状态
//-------------------------------------------------------------------
void huidu_update(void)
{
    uint8 i;
    int32 sum_active   = 0;
    int32 sum_weighted = 0;

    // ----- 1. 读取原始电平并防抖 -----
    #define DEBOUNCE_CNT  2               // 连续确认次数
    static uint8 debounce[7] = {0};       // 连续高电平计数
    static uint8 stable[7] = {0};         // 经过防抖的稳定值

    for(i=0; i<7; i++)
    {
        if(gpio_get_level(pins[i]) == HUIDU_LINE_LEVEL)
        {
            if(debounce[i] < DEBOUNCE_CNT) debounce[i]++;
            else stable[i] = 1;
        }
        else
        {
            if(debounce[i] > 0) debounce[i]--;
            else stable[i] = 0;
        }
        g_huidu.raw[i]    = (stable[i] == 1);
        g_huidu.active[i] = stable[i];
        if(stable[i]) sum_active++;
    }
    g_huidu.active_count = sum_active;

    // ----- 2. 加权偏差计算 -----
    if(sum_active == 0)
    {
        // 完全丢线：保持上一次偏差，不更新
        // g_huidu.deviation 不变
    }
    else
    {
        for(i=0; i<7; i++)
            if(g_huidu.active[i])
                sum_weighted += pos_weight[i];

        g_huidu.deviation = (int16)(sum_weighted / (int32)sum_active);
    }

		
		
		
    // ----- 3. 状态识别（优先级从高到低） -----
    uint8_t left_all  = (g_huidu.active[0] && g_huidu.active[1] && g_huidu.active[2]);  // L3+L2+L1
    uint8_t right_all = (g_huidu.active[4] && g_huidu.active[5] && g_huidu.active[6]);  // R1+R2+R3
    uint8_t mid       =  g_huidu.active[3];                                             // MID
    uint8_t left_any  = (g_huidu.active[0] || g_huidu.active[1] || g_huidu.active[2]);
    uint8_t right_any = (g_huidu.active[4] || g_huidu.active[5] || g_huidu.active[6]);

    if(sum_active == 0)
    {
        g_huidu.state = HUIDU_STATE_LOST;
    }
    else if(sum_active == 7)
    {
        g_huidu.state = HUIDU_STATE_CROSS;
    }
    else if(left_all && mid && right_any)       // 左侧三路 + 中间 + 右侧任意 → 左岔路
    {
        g_huidu.state = HUIDU_STATE_T_LEFT;
    }
    else if(right_all && mid && left_any)       // 右侧三路 + 中间 + 左侧任意 → 右岔路
    {
        g_huidu.state = HUIDU_STATE_T_RIGHT;
    }
    else if(left_any && !mid && !right_any)     // 仅有左侧传感器 → 急右弯（线在左）
    {
        g_huidu.state = HUIDU_STATE_SHARP_RIGHT;  // 需要右转才跟上线
    }
    else if(right_any && !mid && !left_any)     // 仅有右侧传感器 → 急左弯（线在右）
    {
        g_huidu.state = HUIDU_STATE_SHARP_LEFT;
    }
    else
    {
        g_huidu.state = HUIDU_STATE_NORMAL;
    }
}

//-------------------------------------------------------------------
// 便捷函数
//-------------------------------------------------------------------
int16 huidu_get_deviation(void)
{
    return g_huidu.deviation;
}

uint8 huidu_get_state(void)
{
    return g_huidu.state;
}

uint8 huidu_is_lost(void)
{
    return (g_huidu.state == HUIDU_STATE_LOST);
}

uint8 huidu_is_cross(void)
{
    return (g_huidu.state == HUIDU_STATE_CROSS);
}

//-------------------------------------------------------------------
// OLED 四行调试显示
//-------------------------------------------------------------------
void huidu_debug_oled_4line(int16 speedL, int16 speedR)
{
    char buf[17];
    // 行1：左右速度
    snprintf(buf, 17, "L:%5d R:%5d", speedL, speedR);
    OLED_ShowString(1, 1, buf);

    // 行2：偏差值 + 状态代号
    const char *state_short[] = {"N","L","C","TL","TR","SL","SR","BK"};
    snprintf(buf, 17, "Dev:%5d St:%s", g_huidu.deviation, state_short[g_huidu.state]);
    OLED_ShowString(2, 1, buf);

    // 行3：7 路传感器状态（1/0 显示）
    for(uint8 i=0; i<7; i++)
    {
        buf[i*2]   = g_huidu.active[i] ? '#' : '.';
        buf[i*2+1] = ' ';
    }
    buf[14] = '\0';
    OLED_ShowString(3, 1, buf);

    // 行4：传感器数量
    snprintf(buf, 17, "Active: %d", g_huidu.active_count);
    OLED_ShowString(4, 1, buf);
}