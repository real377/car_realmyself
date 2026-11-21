/*********************************************************************************************************************
* 文件名称          main
* 适用平台          MSPM0G3507
* 说明             主函数：初始化 → PIT 中断 → 主循环仅执行调试显示
*                  PIT 中断（10ms）：速度计算 + 灰度更新 + 循迹 + PID 控制
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "huidu.h"

/* ==================== 全局 PID 对象 ==================== */
PID_TypeDef g_pidTrace;         // 循迹转向环

/* ==================== 控制参数宏 ==================== */
#define CTRL_PERIOD_MS      5              // 控制周期 10ms
#define BASE_SPEED_TARGET   10              // 基础目标速度（脉冲/控制周期）

/* ==================== 测试模式开关 ==================== */
#define TEST_MODE_BASIC     1               // 1=基础循迹测试，0=完整状态机

//-------------------------------------------------------------------------------------------------------------------
// PIT 中断回调函数（每 10ms 执行一次）
//-------------------------------------------------------------------------------------------------------------------
void control_isr(uint32 state, void *ptr)
{
    // 1. 计算当前速度（内部清零脉冲计数）
    encoder_speed_calc(CTRL_PERIOD_MS);

    // 2. 更新灰度传感器数据
//    huidu_update();

    // 3. 循迹 PID：将偏差归零
    float steer = pid_calc(&g_pidTrace, (float)huidu_get_deviation());

    // 4. 速度 PID：维持恒定速度
    float outL = pid_calc(&g_pidSpeedL, (float)g_speedL_now);
    float outR = pid_calc(&g_pidSpeedR, (float)g_speedR_now);

    // 5. 融合输出
    int32 dutyL = (int32)outL + (int32)steer;
    int32 dutyR = (int32)outR - (int32)steer;

    // 6. 特殊状态处理（可按需扩展）
    if(huidu_is_lost())
    {
        // 短暂的丢线可以保持上一次的输出，
        // 若连续丢线超过 200ms，则强制停车（这里预留停机标志）
        static uint8 lost_cnt = 0;
        lost_cnt++;
        if(lost_cnt > (200 / CTRL_PERIOD_MS))
        {
            motor_stop();
            return;
        }
    }
    else
    {
        // 恢复正常后清除丢线计数
        static uint8 lost_cnt = 0;  // 实际上应放到函数外 static，但这里仅为示意
        lost_cnt = 0;
    }

    // 输出电机
    motor_set_speed(dutyL, dutyR);
}

//-------------------------------------------------------------------------------------------------------------------
// 主函数
//-------------------------------------------------------------------------------------------------------------------
int main(void)
{
    // ---------- 系统初始化 ----------
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();

    // 外设初始化
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Init...");

    motor_init();
    encoder_init();
    huidu_init();

    // ---------- PID 参数初始化 ----------
    // 速度环 PID（增量式）—— 使用你之前调好的参数
    pid_init(&g_pidSpeedL, PID_MODE_INCREMENTAL,
             1.0f, 0.5f, 0.0f,              // Kp, Ki, Kd（请替换为你的实际参数）
             -(float)BASE_SPEED_TARGET,       // 目标速度
             3000.0f,                        // 输出限幅
             3000.0f,                        // 积分限幅
             1.0f);                          // 死区

    pid_init(&g_pidSpeedR, PID_MODE_INCREMENTAL,
             1.0f, 0.5f, 0.0f,
             -(float)BASE_SPEED_TARGET,
             3000.0f, 3000.0f, 1.0f);

    // 循迹 PID（位置式）
    pid_init(&g_pidTrace, PID_MODE_POSITIONAL,
             0.8f, 0.02f, 0.5f,             // Kp, Ki, Kd（需实测调整）
             0.0f,                           // 目标偏差 = 0
             0.0f,                        // 输出限幅（转向量上限）
             500.0f,                         // 积分限幅
             20.0f);                         // 死区

    // ---------- PIT 定时中断初始化 ----------
    pit_ms_init(PIT_TIM_G6, CTRL_PERIOD_MS, control_isr, NULL);
    // 注意：MSPM0G3507 的 PIT 回调函数签名需要匹配 void_callback_uint32_ptr
    // 若 control_isr 不符合，需包装一下，这里示意接口

    OLED_Clear();

    // ---------- 主循环 ----------
    while(1)
    {
        // 四行 OLED 调试显示
        huidu_debug_oled_4line (g_speedL_now, g_speedR_now);

        system_delay_ms(50);  // 刷新率 20Hz（也可以更快）
    }
}