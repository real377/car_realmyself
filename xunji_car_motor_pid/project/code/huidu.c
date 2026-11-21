/*********************************************************************************************************************
* 文件名称          huidu
* 适用平台          MSPM0G3507
* 说明             7 路灰度传感器驱动
*                  - 加权偏差计算
*                  - 路口模式识别
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "huidu.h"

/* ==================== 全局数据定义 ==================== */
HuiduData_TypeDef g_huidu = {0};

/* ==================== 每个传感器的位置权重 ==================== */
// 布局：  L3    L2    L1    MID   R1    R2    R3
// 权重：  -3    -2    -1     0    +1    +2    +3
// 偏差 = Σ(active[i] × weight[i]) / Σ(active[i])
// 结果再放大 1000 倍以便 PID 使用
static const int8 huidu_weight[7] = {-3, -2, -1, 0, 1, 2, 3};

/* ==================== GPIO 引脚数组（便于循环读取） ==================== */
static const gpio_pin_enum huidu_pins[7] =
{
    HUIDU_L3_PIN,   // [0] L3
    HUIDU_L2_PIN,   // [1] L2
    HUIDU_L1_PIN,   // [2] L1
    HUIDU_MID_PIN,  // [3] MID
    HUIDU_R1_PIN,   // [4] R1
    HUIDU_R2_PIN,   // [5] R2
    HUIDU_R3_PIN    // [6] R3
};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度传感器 GPIO 初始化
// 备注信息     7 路均配置为输入 + 下拉电阻
//             检测到线 = 低电平（HUIDU_LINE_LEVEL=0），无检测 = 高电平
//-------------------------------------------------------------------------------------------------------------------
void huidu_init(void)
{
    uint8 i;
    for(i = 0; i < 7; i++)
    {
        gpio_init(huidu_pins[i], GPI, GPIO_LOW, GPI_PULL_DOWN);
    }

    // 清零全局数据
    memset(&g_huidu, 0, sizeof(HuiduData_TypeDef));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新灰度传感器数据
// 备注信息     每个控制周期（如 5~10ms）调用一次
//             1. 读取 7 路原始电平
//             2. 转换为"检测到线"标志
//             3. 计算加权偏差
//             4. 识别路口状态
//-------------------------------------------------------------------------------------------------------------------
void huidu_update(void)
{
    uint8 i;
    int32 sum_weighted = 0;
    int32 sum_active   = 0;

    // ---------- 第1步：读取原始电平 + 转换 ----------
    g_huidu.active_count = 0;
    for(i = 0; i < 7; i++)
    {
        g_huidu.raw[i] = gpio_get_level(huidu_pins[i]);

        // 判断是否检测到线
        if(g_huidu.raw[i] == HUIDU_LINE_LEVEL)
        {
            g_huidu.active[i] = 1;
            g_huidu.active_count++;
        }
        else
        {
            g_huidu.active[i] = 0;
        }
    }

    // ---------- 第2步：计算加权偏差 ----------
    if(g_huidu.active_count > 0)
    {
        for(i = 0; i < 7; i++)
        {
            if(g_huidu.active[i])
            {
                sum_weighted += huidu_weight[i];
                sum_active   += 1;
            }
        }
        // 归一化：偏差 = 加权平均 × 1000
        // 例：只有 MID(权重0) → 偏差=0
        //     只有 R1(权重+1) → 偏差=+1000
        //     只有 L1(权重-1) → 偏差=-1000
        //     L1+MID(权重-1,0) → 偏差=-500
        g_huidu.deviation = (int16)((sum_weighted * 1000) / (int32)sum_active);

        // 保存上一次有效偏差（丢线时复用）
        static int16 last_valid_dev = 0;
        last_valid_dev = g_huidu.deviation;
    }
    else
    {
        // 完全丢线：保持上一次偏差（或置0由上层决定）
        // g_huidu.deviation 保持不变，由状态标志通知上层
    }

    // ---------- 第3步：路口/状态识别 ----------
    // 优先级从高到低判断

    // 全黑 = 十字路口（7 路全部检测到线）
    if(g_huidu.active_count >= 7)
    {
        g_huidu.state = HUIDU_STATE_CROSS;
    }
    // 左 T 字路口：左侧3路 + 中间 + 右侧至少1路 都有线
    // 典型模式：L3+L2+L1+MID+R1 都存在
    else if(g_huidu.active[0] && g_huidu.active[1] && g_huidu.active[2] &&
            g_huidu.active[3] &&
            (g_huidu.active[4] || g_huidu.active[5] || g_huidu.active[6]))
    {
        g_huidu.state = HUIDU_STATE_T_LEFT;
    }
    // 右 T 字路口：右侧3路 + 中间 + 左侧至少1路 都有线
    else if(g_huidu.active[4] && g_huidu.active[5] && g_huidu.active[6] &&
            g_huidu.active[3] &&
            (g_huidu.active[0] || g_huidu.active[1] || g_huidu.active[2]))
    {
        g_huidu.state = HUIDU_STATE_T_RIGHT;
    }
    // 左直角弯：只有左侧传感器检测到线，中间和右侧都没有
    else if((g_huidu.active[0] || g_huidu.active[1] || g_huidu.active[2]) &&
            !g_huidu.active[3] &&
            !g_huidu.active[4] && !g_huidu.active[5] && !g_huidu.active[6])
    {
        g_huidu.state = HUIDU_STATE_LEFT_TURN;
        g_huidu.deviation = HUIDU_DEVIATION_MIN;  // 强制最大左偏
    }
    // 右直角弯：只有右侧传感器检测到线
    else if(!g_huidu.active[0] && !g_huidu.active[1] && !g_huidu.active[2] &&
            !g_huidu.active[3] &&
            (g_huidu.active[4] || g_huidu.active[5] || g_huidu.active[6]))
    {
        g_huidu.state = HUIDU_STATE_RIGHT_TURN;
        g_huidu.deviation = HUIDU_DEVIATION_MAX;  // 强制最大右偏
    }
    // 丢线：没有任何传感器检测到线
    else if(g_huidu.active_count == 0)
    {
        g_huidu.state = HUIDU_STATE_LOST;
    }
    // 正常循迹
    else
    {
        g_huidu.state = HUIDU_STATE_NORMAL;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 便捷接口
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     OLED 调试显示
// 备注信息     在 OLED 第3~4行显示7路传感器状态
//             格式：L3 L2 L1 M R1 R2 R3
//             '#' = 检测到线，'.' = 未检测到
//-------------------------------------------------------------------------------------------------------------------
void huidu_debug_oled(void)
{
    static const char *label[7] = {"L3","L2","L1","M ","R1","R2","R3"};
    char buf_line1[17] = {0};   // 第1行：标签
    char buf_line2[17] = {0};   // 第2行：状态

    uint8 i;
    for(i = 0; i < 7; i++)
    {
        // 标签（2字符 + 空格）
        buf_line1[i * 2]     = label[i][0];
        buf_line1[i * 2 + 1] = label[i][1];

        // 状态
        buf_line2[i * 2]     = g_huidu.active[i] ? '#' : '.';
        buf_line2[i * 2 + 1] = ' ';
    }
    buf_line1[14] = '\0';
    buf_line2[14] = '\0';

    OLED_ShowString(3, 1, buf_line1);
    OLED_ShowString(4, 1, buf_line2);

    // 显示偏差值
    OLED_ShowSignedNum(5, 1, g_huidu.deviation, 5);

    // 显示状态
    const char *state_str[] = {
        "NORMAL", "LOST", "CROSS", "T_LEFT",
        "T_RIGHT", "L_TURN", "R_TURN", "BLACK"
    };
    OLED_ShowString(6, 1, (char*)state_str[g_huidu.state]);
}