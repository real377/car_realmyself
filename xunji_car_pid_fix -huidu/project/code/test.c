/*********************************************************************************************************************
* 文件名称          test
* 适用平台          MSPM0G3507
********************************************************************************************************************/

#include "zf_common_headfile.h"
    #define SPEED_CTRL_PERIOD_MS  10

/* ==================== 全局 PID 对象定义 ==================== */
PID_TypeDef g_pidSpeedL;
PID_TypeDef g_pidSpeedR;

/* ==================== 内部状态 ==================== */
static uint8  test_mode       = 0;       // 0=空闲, 1=定速, 2=定距
static uint32 test_tick       = 0;       // 测试计时（ms 累计）
static float  test_target_dist = 0;      // 目标距离

//-------------------------------------------------------------------------------------------------------------------
// 测试 1：电机基础测试
// 说明：依次执行 → 正转 2s → 停止 1s → 反转 2s → 停止
// 用户观察电机是否正常转动，方向是否正确
//-------------------------------------------------------------------------------------------------------------------
void test_motor_basic(void)
{
    uint32 delay;

    motor_init();
    encoder_init();
		OLED_Init();
    OLED_ShowString(1, 1, "Motor Test");

    // 正转 50% 占空比
    OLED_ShowString(2, 1, "Forward 50%");
    motor_set_speed(5000, 5000);
    for(delay = 0; delay < 2000000; delay++) { __NOP(); }  // 约 2s（需根据主频调整）

    // 停止
    OLED_ShowString(2, 1, "Stop         ");
    motor_stop();
    for(delay = 0; delay < 1000000; delay++) { __NOP(); }

    // 反转 50%
    OLED_ShowString(2, 1, "Reverse 50%  ");
    motor_set_speed(-5000, -5000);
    for(delay = 0; delay < 2000000; delay++) { __NOP(); }

    // 停止
    motor_stop();
    OLED_ShowString(2, 1, "Test Done!   ");
}

//-------------------------------------------------------------------------------------------------------------------
// 测试 2：编码器测试
// 说明：初始化编码器，在 OLED 上实时显示脉冲计数
// 用户手动推车，观察数值变化是否正确（前进+、后退-）
// 左右轮分别显示
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_read(void)
{
    encoder_init();
		OLED_Init();
	
	
	
    OLED_Clear();
    OLED_ShowString(1, 1, "Encoder Test");
    OLED_ShowString(2, 1, "L:");
    OLED_ShowString(3, 1, "R:");
	
	

    while(1)
    {
        // 实时显示累计脉冲（低 16 位）
        OLED_ShowSignedNum(2, 4, (int32)g_encL_count, 6);
        OLED_ShowSignedNum(3, 4, (int32)g_encR_count, 6);
        // 也显示总里程
        OLED_ShowSignedNum(4, 1, (int32)g_encL_pulse_sum, 8);
        OLED_ShowSignedNum(5, 1, (int32)g_encR_pulse_sum, 8);

        system_delay_ms(50);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 测试 3：PID 速度闭环测试
// 说明：初始化速度 PID，以恒定目标速度行驶指定时长
// 在 OLED 上显示目标速度 vs 实际速度，方便调参
// 参数：target_speed  目标速度（脉冲/控制周期）
//       duration_ms   测试时长
//-------------------------------------------------------------------------------------------------------------------
void test_pid_speed(int16 target_speed, uint32 duration_ms)
{
    motor_init();
    encoder_init();
		OLED_Init();
	
	
    // 初始化速度 PID（位置式）
    // 参数建议起点：Kp=3.0, Ki=0.5, Kd=0, out_limit=7000, integral_limit=2000, dead_zone=1
    pid_init(&g_pidSpeedL, PID_MODE_POSITIONAL,
             6144, 2000, 0,
             (float)target_speed, 6000, 4000, 1);
    pid_init(&g_pidSpeedR, PID_MODE_POSITIONAL,
             6144, 2000, 0,
             (float)target_speed, 6000, 4000, 1);


		pit_ms_init(PIT_TIM_G6, SPEED_CTRL_PERIOD_MS, test_speed_control_callback, NULL);
	
	
	
    OLED_Clear();

    test_mode = 1;
    test_tick = 0;

    // 在 PIT 中断中调用 test_speed_control_callback()
    // 此处假设已通过 pit_ms_init(PIT_TIM_G6, SPEED_CTRL_PERIOD_MS, callback, NULL) 注册

    while(test_tick < duration_ms)
    {
				OLED_ShowSignedNum(3, 3, g_speedL_now, 4);        // 左实际
				OLED_ShowSignedNum(4, 3, g_speedR_now, 4);        // 右实际
				OLED_ShowSignedNum(1, 1, (int16)g_pidSpeedL.output_val, 4); // 左输出
				OLED_ShowSignedNum(2, 1, (int16)g_pidSpeedR.output_val, 4); // 右输出
//				OLED_ShowSignedNum(1, 3, test_target_speed, 4); // 
			
			
				system_delay_ms(100);
    }

    // 测试结束，停车
    test_mode = 0;
    motor_stop();
    pid_reset(&g_pidSpeedL);
    pid_reset(&g_pidSpeedR);
    OLED_ShowString(4, 1, "Done!");
}

//-------------------------------------------------------------------------------------------------------------------
// 测试 4：综合行驶测试（定距）
// 说明：基于编码器里程 + PID 速度环，行驶指定距离后自动停车
//-------------------------------------------------------------------------------------------------------------------
void test_distance_run(float target_distance_mm, int16 target_speed)
{
    motor_init();
    encoder_init();
		OLED_Init();

	
    // 初始化速度 PID（位置式）
    // 参数建议起点：Kp=3.0, Ki=0.5, Kd=0, out_limit=7000, integral_limit=2000, dead_zone=1
    pid_init(&g_pidSpeedL, PID_MODE_POSITIONAL,
             6144, 2000, 0,
             (float)target_speed, 6000, 4000, 1);
    pid_init(&g_pidSpeedR, PID_MODE_POSITIONAL,
             6144, 2000, 0,
             (float)target_speed, 6000, 4000, 1);

    // 清零里程
    g_encL_pulse_sum = 0;
    g_encR_pulse_sum = 0;
    g_distance_mm = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, "Dist Test");
    OLED_ShowString(2, 1, "Target:");
    OLED_ShowNum(2, 8, (uint32)target_distance_mm, 4);
    OLED_ShowString(3, 1, "Now:");

    test_mode = 2;
    test_target_dist = target_distance_mm;

    while(g_distance_mm < test_target_dist)
    {
        OLED_ShowNum(3, 5, (uint32)g_distance_mm, 4);
        system_delay_ms(100);
    }

    // 到达目标距离，停车
    test_mode = 0;
    motor_stop();
    pid_reset(&g_pidSpeedL);
    pid_reset(&g_pidSpeedR);
    OLED_ShowString(5, 1, "Arrived!");
}

//-------------------------------------------------------------------------------------------------------------------
// 速度控制回调函数（在 PIT 中断中每 10ms 调用一次）
// 用法：pit_ms_init(PIT_TIM_G6, 10, test_pit_callback, NULL);
// 在回调中调用此函数
//-------------------------------------------------------------------------------------------------------------------
void test_speed_control_callback(uint32 state, void *ptr)
{
    // 1. 计算当前速度
    encoder_speed_calc(SPEED_CTRL_PERIOD_MS);

    // 2. 根据测试模式执行
    if(test_mode == 1 || test_mode == 2)
    {
        // PID 计算
        float outL = pid_calc(&g_pidSpeedL, (float)g_speedL_now);
        float outR = pid_calc(&g_pidSpeedR, (float)g_speedR_now);

        // 输出到电机
        motor_set_speed(-(int32)outL, -(int32)outR);
    }

    // 3. 计时
    if(test_mode == 1)
    {
        test_tick += SPEED_CTRL_PERIOD_MS;
    }
}