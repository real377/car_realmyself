/*********************************************************************************************************************
* 文件名称          encoder
* 适用平台          MSPM0G3507
* 说明             基于 EXTI 的编码器速度检测
*                  - 双边沿触发 + 仅处理高电平 = 2 倍频解码
*                  - 在 PIT 定时中断中周期性调用 encoder_speed_calc()
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "encoder.h"

/* ==================== 全局变量定义 ==================== */
volatile int16_t  g_encL_count      = 0;
volatile int16_t  g_encR_count      = 0;
volatile int32_t  g_encL_pulse_sum  = 0;
volatile int32_t  g_encR_pulse_sum  = 0;

int16_t  g_speedL_now   = 0;
int16_t  g_speedR_now   = 0;
float    g_distance_mm  = 0.0f;

/* ==================== 中断回调声明 ==================== */
static void L_PIN1_handler(uint32 state, void *ptr);
static void L_PIN2_handler(uint32 state, void *ptr);
static void R_PIN1_handler(uint32 state, void *ptr);
static void R_PIN2_handler(uint32 state, void *ptr);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 备注信息     4 路 EXTI 均配置为双边沿触发
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    exti_init(ENCODER_L_PIN1, EXTI_TRIGGER_BOTH, L_PIN1_handler, NULL);
    exti_init(ENCODER_L_PIN2, EXTI_TRIGGER_BOTH, L_PIN2_handler, NULL);
    exti_init(ENCODER_R_PIN1, EXTI_TRIGGER_BOTH, R_PIN1_handler, NULL);
    exti_init(ENCODER_R_PIN2, EXTI_TRIGGER_BOTH, R_PIN2_handler, NULL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清零脉冲计数（在定时中断中，读完速度后调用）
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear(void)
{
    g_encL_count = 0;
    g_encR_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度计算（在 PIT 定时中断中调用）
// 参数说明     period_ms   控制周期（ms），如 10ms
// 备注信息     速度单位 = 脉冲数 / 控制周期
//             如需转换为 mm/s：speed * MM_PER_PULSE * 1000 / period_ms
//-------------------------------------------------------------------------------------------------------------------
void encoder_speed_calc(uint16 period_ms)
{
    // 滤波缓冲（简单均值滤波）
    static int16_t bufL[4] = {0}, bufR[4] = {0};
    static uint8_t buf_idx = 0;
    int32_t sumL = 0, sumR = 0;
    uint8_t i;

    // 存入当前值
    bufL[buf_idx] = g_encL_count;
    bufR[buf_idx] = g_encR_count;
    buf_idx = (buf_idx + 1) & 0x03;  // 循环索引 0~3

    // 4 次滑动均值
    for(i = 0; i < 4; i++)
    {
        sumL += bufL[i];
        sumR += bufR[i];
    }
    g_speedL_now = (int16_t)(sumL >> 2);  // 除以 4
    g_speedR_now = (int16_t)(sumR >> 2);

    // 里程累计
    g_distance_mm += (g_speedL_now + g_speedR_now) * 0.5f * MM_PER_PULSE;

    encoder_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 左编码器 A 相中断
//-------------------------------------------------------------------------------------------------------------------
static void L_PIN1_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(ENCODER_L_PIN1))  // 仅处理上升沿（2 倍频）
    {
        if(gpio_get_level(ENCODER_L_PIN2))
        {
            g_encL_count++;
            g_encL_pulse_sum++;
        }
        else
        {
            g_encL_count--;
            g_encL_pulse_sum--;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 左编码器 B 相中断
//-------------------------------------------------------------------------------------------------------------------
static void L_PIN2_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(ENCODER_L_PIN2))
    {
        if(gpio_get_level(ENCODER_L_PIN1))
        {
            g_encL_count--;
            g_encL_pulse_sum--;
        }
        else
        {
            g_encL_count++;
            g_encL_pulse_sum++;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 右编码器 A 相中断
//-------------------------------------------------------------------------------------------------------------------
static void R_PIN1_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(ENCODER_R_PIN1))
    {
        if(gpio_get_level(ENCODER_R_PIN2))
        {
            g_encR_count++;
            g_encR_pulse_sum++;
        }
        else
        {
            g_encR_count--;
            g_encR_pulse_sum--;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 右编码器 B 相中断
//-------------------------------------------------------------------------------------------------------------------
static void R_PIN2_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(ENCODER_R_PIN2))
    {
        if(gpio_get_level(ENCODER_R_PIN1))
        {
            g_encR_count--;
            g_encR_pulse_sum--;
        }
        else
        {
            g_encR_count++;
            g_encR_pulse_sum++;
        }
    }
}