/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 文件名称          main
* 功能说明          双路42步进电机测试——PIT中断脉冲 + 梯形加减速 + 步数计数
*                   电机1: 正转2圈 → 停1秒 → 反转2圈 → 停1秒 → 循环
*                   电机2: 正转1圈 → 停1秒 → 反转1圈 → 停1秒 → 循环
* 适用平台          MSPM0G3507 + D36A驱动板 + 42步进电机
* 微步设置          16细分 (3200 脉冲/圈)
********************************************************************************************************************/

#include "zf_common_headfile.h"

// **************************** 主函数 ****************************

int main(void)
{
		char buffer[40];
    /* 系统初始化 */
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();
		pit_ms_init(PIT_TIM_G6, 10, test_speed_control_callback, NULL);
	
	
//		test_pid_speed(10, 10000);
		test_distance_run(1000, 10);



/* 主循环 */
    while (1)
    {
			
//        sprintf(buffer, "%d/%d C:%d", L_pulse_sum,R_pulse_sum,R_count);
//        OLED_ShowString(1, 1, buffer);



			
    }
}