#include "zf_common_headfile.h"

void L_PIN1_exti_handler(uint32 state, void *ptr);
void L_PIN2_exti_handler(uint32 state, void *ptr);
void R_PIN1_exti_handler(uint32 state, void *ptr);
void R_PIN2_exti_handler(uint32 state, void *ptr);

// 原有变量（每周期清零的脉冲数）
int16_t L_count = 0, R_count = 0, encoder_flag = 0;

// 新增总脉冲数（不清零，专用于路程计算）
int32_t L_pulse_sum = 0;
int32_t R_pulse_sum = 0;

void ctimer_clean(void)
{
    L_count = 0;	
    R_count = 0;
}

void encoder_init()
{
    exti_init(L_PIN1, EXTI_TRIGGER_BOTH, L_PIN1_exti_handler, NULL);
    exti_init(L_PIN2, EXTI_TRIGGER_BOTH, L_PIN2_exti_handler, NULL);
    exti_init(R_PIN1, EXTI_TRIGGER_BOTH, R_PIN1_exti_handler, NULL);
    exti_init(R_PIN2, EXTI_TRIGGER_BOTH, R_PIN2_exti_handler, NULL);
}

void L_PIN1_exti_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(L_PIN1)) 
    {
        uint8 pin2 = gpio_get_level(L_PIN2);
        if (pin2) 
        {
            L_count++;
            L_pulse_sum++;      // 累计总脉冲
        }
        else 
        {
            L_count--;
            L_pulse_sum--;      // 累计总脉冲
        }
    }
}

void L_PIN2_exti_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(L_PIN2)) 
    {
        uint8 pin1 = gpio_get_level(L_PIN1);
        if (pin1) 
        {
            L_count--;
            L_pulse_sum--;      // 累计总脉冲
        }
        else 
        {
            L_count++;
            L_pulse_sum++;      // 累计总脉冲
        }
    }
}

void R_PIN1_exti_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(R_PIN1)) 
    {
        uint8 pin2 = gpio_get_level(R_PIN2);
        if (pin2) 
        {
            R_count++;
            R_pulse_sum++;      // 累计总脉冲
        }
        else 
        {
            R_count--;
            R_pulse_sum--;      // 累计总脉冲
        }
    }
}

void R_PIN2_exti_handler(uint32 state, void *ptr)
{
    if(gpio_get_level(R_PIN2)) 
    {
        uint8 pin1 = gpio_get_level(R_PIN1);
        if (pin1) 
        {
            R_count--;
            R_pulse_sum--;      // 累计总脉冲
        }
        else 
        {
            R_count++;
            R_pulse_sum++;      // 累计总脉冲
        }
    }
}