/*********************************************************************************************************************
* 文件名称          motor
* 适用平台          MSPM0G3507
********************************************************************************************************************/

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 备注信息     方向控制引脚 → 推挽输出；PWM → 频率 17kHz，初始占空比 0
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    gpio_init(DIR_L1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_L2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_R1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_R2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0);
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左电机控制
// 参数说明     duty    -PWM_DUTY_MAX ~ +PWM_DUTY_MAX，正=正转，负=反转
//-------------------------------------------------------------------------------------------------------------------
void motor_set_left(int32 duty)
{
    if(duty >  PWM_DUTY_MAX) duty =  PWM_DUTY_MAX;
    if(duty < -PWM_DUTY_MAX) duty = -PWM_DUTY_MAX;

    if(duty >= 0)
    {
        gpio_set_level(DIR_L1, 0);
        gpio_set_level(DIR_L2, 1);
        pwm_set_duty(PWM_L, (uint32)duty);
    }
    else
    {
        gpio_set_level(DIR_L1, 1);
        gpio_set_level(DIR_L2, 0);
        pwm_set_duty(PWM_L, (uint32)(-duty));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     右电机控制
//-------------------------------------------------------------------------------------------------------------------
void motor_set_right(int32 duty)
{
    if(duty >  PWM_DUTY_MAX) duty =  PWM_DUTY_MAX;
    if(duty < -PWM_DUTY_MAX) duty = -PWM_DUTY_MAX;

    if(duty >= 0)
    {
        gpio_set_level(DIR_R1, 0);
        gpio_set_level(DIR_R2, 1);
        pwm_set_duty(PWM_R, (uint32)duty);
    }
    else
    {
        gpio_set_level(DIR_R1, 1);
        gpio_set_level(DIR_R2, 0);
        pwm_set_duty(PWM_R, (uint32)(-duty));
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     同时设置两电机
//-------------------------------------------------------------------------------------------------------------------
void motor_set_speed(int32 dutyL, int32 dutyR)
{
    motor_set_left(-dutyL);//纠正方向
    motor_set_right(-dutyR);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机停止（刹车）
//-------------------------------------------------------------------------------------------------------------------
void motor_stop(void)
{
    gpio_set_level(DIR_L1, 0);
    gpio_set_level(DIR_L2, 0);
    gpio_set_level(DIR_R1, 0);
    gpio_set_level(DIR_R2, 0);
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
}