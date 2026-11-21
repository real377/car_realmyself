/*****************************************************************************
 * 文件名称          main.c
 * 适用平台          MSPM0G3507
 * 说明             灰度循迹测试主函数
 *                  - 10ms 控制周期（PIT 中断）
 *                  - 根据 gray 差速直接控制左右电机
 *                  - OLED 显示调试信息
 *****************************************************************************/

#include "zf_common_headfile.h"   // 包含逐飞平台公共头文件
#include "motor.h"
#include "encoder.h"
#include "gray.h"
#include "OLED.h"

/* ==================== 控制参数 ==================== */
#define CONTROL_PERIOD_MS  10      // 控制周期（ms）
#define BASE_SPEED         1500    // 基础速度（0 ~ PWM_DUTY_MAX，例如 4000 对应 40% 占空比）

/* ==================== 运行状态 ==================== */
static uint8_t g_run_flag = 1;     // 1 = 运行，0 = 停止

/* ==================== PIT 中断回调 ==================== */
void pit_control_callback(uint32_t state, void *ptr)
{
    // 1. 编码器数据更新（若不需要速度反馈可注释）
    encoder_speed_calc(CONTROL_PERIOD_MS);

    // 2. 灰度传感器更新
    gray_update();

    // 3. 电机控制
    if (g_run_flag)
    {
        int16_t turn_diff = gray_get_turn_diff();   // 获取限幅后的差速 [-500, 500]

        // 差速转向：左轮减速，右轮加速（或相反，取决于实际安装）
        int32_t left_duty  = BASE_SPEED + turn_diff;
        int32_t right_duty = BASE_SPEED - turn_diff;

//        motor_set_speed(left_duty, right_duty);
    }
    else
    {
        motor_stop();
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    /* ---------- 平台初始化（根据实际开发板调整） ---------- */
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>

    debug_init();           // 初始化延时函数

    /* ---------- 外设初始化 ---------- */
    motor_init();                  // 电机驱动
    encoder_init();                // 编码器（若需速度闭环则保留）
    gray_init();                   // 灰度传感器
    OLED_Init();                   // OLED 显示屏

    OLED_ShowString(1, 1, "Gray Test");
    system_delay_ms(500);

    /* ---------- 设置控制定时器 ---------- */
    // 使用逐飞库的 PIT 定时器，每 10ms 触发一次 pit_control_callback
    pit_ms_init(PIT_TIM_G6, CONTROL_PERIOD_MS, pit_control_callback, NULL);

    /* ---------- 启动界面 ---------- */
    OLED_Clear();
    OLED_ShowString(1, 1, "Base Speed:");
    OLED_ShowNum(1, 12, BASE_SPEED, 4);

    /* ---------- 主循环（仅用于调试显示） ---------- */
    while (1)
    {
        // 在 OLED 上显示灰度传感器状态、偏差、差速
        gray_debug_oled();

        // 延时，降低 OLED 刷新频率
        system_delay_ms(100);
    }
}