#include "zf_common_headfile.h"
#include "SR04.h"

// ============ 使用空闲TIM_G6做硬件微秒计时（TIM_G6没被占用，无冲突） ============
#define SR04_HWTIMER_INDEX   TIM_G6

// ============ 全局变量 ============
volatile uint8_t  sr04_measure_done = 0;    // 测量完成标志
volatile uint32_t sr04_high_time_us = 0;    // 高电平时间(微秒)

int Distance = 0;                           // 距离值 (cm)

// 一阶指数滑动滤波：新值权重75% + 旧值权重25%
// → 响应比5次平均快3倍以上，同时能滤掉±1cm的小抖动
static int sr04_last_dist = 0;
static uint8_t sr04_first_measure = 1;

static int sr04_filter_push(int new_val)
{
    if (sr04_first_measure) { sr04_first_measure = 0; sr04_last_dist = new_val; return new_val; }
    // 75%新值 + 25%旧值 = (新值*3 + 旧值 + 2)/4（+2是四舍五入）
    sr04_last_dist = (new_val * 3 + sr04_last_dist + 2) / 4;
    return sr04_last_dist;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介   获取 SR04 超声波距离 (cm) —— 硬件计时器+极速响应版
//-------------------------------------------------------------------------------------------------------------------
int Get_SR04_Distance(void)
{
    uint32_t time_us = 0;
    uint32_t timeout_flag = 0;
    int raw_dist = 0;
    
    timer_clear(SR04_HWTIMER_INDEX);
    timer_start(SR04_HWTIMER_INDEX);
    
    // 1. Trig 精确15us高脉冲触发
    gpio_set_level(SR04_TRIG_PIN, GPIO_HIGH);
    while (timer_get(SR04_HWTIMER_INDEX) < 15);
    gpio_set_level(SR04_TRIG_PIN, GPIO_LOW);
    
    // 2. 等 ECHO 变高，超时 3000us=3ms（40cm只需~2333us，留余量即可）
    timer_clear(SR04_HWTIMER_INDEX);
    timeout_flag = 0;
    // 如果一上来ECHO就已经是高（异常状态），直接跳过不白等
    if (gpio_get_level(SR04_ECHO_PIN) == GPIO_HIGH) { timeout_flag = 2; }
    else
    {
        while (gpio_get_level(SR04_ECHO_PIN) == GPIO_LOW)
        {
            if (timer_get(SR04_HWTIMER_INDEX) >= 3000) { timeout_flag = 1; break; }
        }
    }
    if (timeout_flag == 1 || timeout_flag == 2)
    {
        timer_stop(SR04_HWTIMER_INDEX);
        sr04_high_time_us = 0;
        if (timeout_flag == 1) Distance = sr04_filter_push(40);  // 真正超时→无障碍物
        else                  Distance = sr04_last_dist;        // ECHO异常→保留上次值
        sr04_measure_done = 1;
        return Distance;
    }
    
    // 3. ECHO 高电平宽度计时，超时 3000us=3ms（40cm只需要2333us）
    timer_clear(SR04_HWTIMER_INDEX);
    timeout_flag = 0;
    while (gpio_get_level(SR04_ECHO_PIN) == GPIO_HIGH)
    {
        if (timer_get(SR04_HWTIMER_INDEX) >= 3000) { timeout_flag = 1; break; }
    }
    time_us = (uint32_t)timer_get(SR04_HWTIMER_INDEX);
    timer_stop(SR04_HWTIMER_INDEX);
    
    sr04_high_time_us = time_us;
    sr04_measure_done = 1;
    
    // 4. SR04官方标准换算：dist(cm) = time_us / 58
    //    2cm ? 117us   10cm ? 583us   40cm ? 2333us
    if (timeout_flag || time_us > 2500)         raw_dist = 40;
    else if (time_us < 117)                      raw_dist = 2;
    else
    {
        raw_dist = (int)(time_us / 58);
        if (raw_dist < 2)  raw_dist = 2;
        if (raw_dist > 40) raw_dist = 40;
    }
    
    // 5. 极速指数滤波：响应快+不跳变
    Distance = sr04_filter_push(raw_dist);
    return Distance;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介   SR04 超声波模块初始化
//-------------------------------------------------------------------------------------------------------------------
void Sr04_All_Init(void)
{
    gpio_init(SR04_TRIG_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SR04_ECHO_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    
    timer_init(SR04_HWTIMER_INDEX, TIMER_US);
    timer_clear(SR04_HWTIMER_INDEX);
    timer_stop(SR04_HWTIMER_INDEX);
    
    gpio_set_level(SR04_TRIG_PIN, GPIO_LOW);
    system_delay_ms(30);
    
    sr04_last_dist = 0;
    sr04_first_measure = 1;
}

// EXTI 回调保留兼容声明，不会被调用
void sr04_echo_exti_callback(uint32 event, void *ptr)
{
    (void)event; (void)ptr;
}


void LightSensor_Init(void)
{
	gpio_init(B11, GPI, GPIO_HIGH, GPI_FLOATING_IN);
}

int LightSensor_Get(void)
{
	return gpio_get_level(B11);
}
