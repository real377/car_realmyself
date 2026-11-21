/*****************************************************************************
 * 文件名称          main.c
 * 适用平台          MSPM0G3507
 * 说明             灰度循迹 + 定点 PID 闭环控制（无浮点，低运算量）
 *                  - 使用 pid.c（纯整数 Q10 格式）
 *                  - 使用 gray.c 获取偏差
 *                  - 使用 IPS200 液晶屏显示调试信息（替代 OLED）
 *                  - 10ms PIT 中断完成控制
 *****************************************************************************/

#include "zf_common_headfile.h"   // 逐飞平台公共头文件
#include "motor.h"
#include "encoder.h"
#include "gray.h"
#include "pid.h"           // 定点整数 PID（Q10 格式）
#include "zf_device_ips200.h"   // IPS200 液晶驱动

/* ==================== 控制参数 ==================== */
#define CONTROL_PERIOD_MS       10      // 控制周期（ms）

#define BASE_SPEED              1500    // 基础速度（0 ~ PWM_DUTY_MAX）
                                        // 建议不超过 PWM_DUTY_MAX - MAX_TURN_DIFF

#define MAX_TURN_DIFF           2000     // 转向差速限幅（与 gray.h 中 GRAY_MAX_TURN_DIFF 保持一致）

/* ==================== 转向 PID 参数（Q10 定点格式，真实值 = 参数 / 1024） ==================== */
// Kp：偏差 3000 → P项输出 = 170 * 3000 / 1024 ≈ 498，接近限幅 500
// Kd：抑制震荡，建议 300~800
#define STEER_KP_Q10            300     // 实际 
#define STEER_KI_Q10            0       // 循迹一般不用积分
#define STEER_KD_Q10            500     // 实际 Kd ≈ 0.488
#define STEER_INTEGRAL_LIMIT    3000    // 积分限幅（即使用不到，但必须填）

/* ==================== 速度闭环开关（0=关闭, 1=开启） ==================== */
#define ENABLE_SPEED_LOOP       1       // 初期调试建议关闭

/* ==================== 速度 PID 参数（Q10 定点格式） ==================== */
#define SPEED_KP_Q10            2000    // 实际 Kp ≈ 1.95
#define SPEED_KI_Q10            500     // 实际 Ki ≈ 0.49
#define SPEED_KD_Q10            0
#define SPEED_INTEGRAL_LIMIT    5000
#define SPEED_OUT_LIMIT         8000    // 速度环输出限幅（PWM 占空比）

/* ==================== 目标速度（脉冲/10ms，仅速度闭环使用） ==================== */
#define TARGET_SPEED            5      // 需根据实际编码器脉冲数调整

/* ==================== 全局 PID 对象 ==================== */
static PID_TypeDef g_pid_steer;         // 转向环 PID（灰度偏差 → 差速）
#if ENABLE_SPEED_LOOP
static PID_TypeDef g_pid_speedL;        // 左轮速度环 PID
static PID_TypeDef g_pid_speedR;        // 右轮速度环 PID
#endif

/* ==================== 运行标志（可通过按键切换） ==================== */
static uint8_t g_run = 1;

/* ==================== IPS200 调试显示（在主循环中调用） ==================== */
static void ips200_display(void)
{
    uint8_t i;
    char buf[8];   // 存储传感器状态字符串

    // 每次刷新前清屏（避免残影）
    ips200_clear();

    // ---------- 第1行：七路传感器状态 ----------
    for (i = 0; i < 7; i++)
    {
        buf[i] = g_gray.active[i] ? '#' : '.';
    }
    buf[7] = '\0';
    ips200_show_string(0, 0, "Gray:");
    ips200_show_string(40, 0, buf);

    // ---------- 第2行：灰度偏差 ----------
    ips200_show_string(0, 20, "Dev:");
    ips200_show_int(40, 20, g_gray.deviation, 5);

    // ---------- 第3行：PID 转向差速输出 ----------
    ips200_show_string(0, 40, "Diff:");
    ips200_show_int(40, 40, g_pid_steer.output_val, 4);

    // ---------- 第4行：左轮速度（编码器脉冲/10ms） ----------
    ips200_show_string(0, 60, "L:");
    ips200_show_int(40, 60, g_speedL_now, 5);

    // ---------- 第5行：右轮速度 ----------
    ips200_show_string(0, 80, "R:");
    ips200_show_int(40, 80, g_speedR_now, 5);

    // ---------- 第6行：传感器状态文字 ----------
    const char *state_str[] = {"NORM","LOST","BLK","L_EDG","R_EDG"};
    ips200_show_string(0, 100, "State:");
    ips200_show_string(50, 100, (char*)state_str[g_gray.state]);

    // ---------- 第7行：运行标志 ----------
    ips200_show_string(0, 120, g_run ? "RUN" : "STOP");

    // ---------- 第8行：转向 PID 系数 ----------
    ips200_show_string(0, 140, "Kp:");
    ips200_show_int(30, 140, STEER_KP_Q10, 3);
    ips200_show_string(90, 140, "Kd:");
    ips200_show_int(120, 140, STEER_KD_Q10, 3);
}

/* ==================== PIT 中断回调（每 10ms 执行一次） ==================== */
void pit_control_callback(uint32_t state, void *ptr)
{
    int32_t steer_diff;     // 转向差速（由转向 PID 计算）
    int32_t left_duty;      // 左轮最终占空比
    int32_t right_duty;     // 右轮最终占空比

    /* ---------- 第1步：更新编码器速度 ---------- */
    encoder_speed_calc(CONTROL_PERIOD_MS);

    /* ---------- 第2步：更新灰度传感器数据 ---------- */
    gray_update();

    /* ---------- 第3步：转向 PID 计算（输入灰度偏差，输出转向差速） ---------- */
    steer_diff = pid_calc(&g_pid_steer, (int32_t)gray_get_deviation());

    /* ---------- 第4步：电机控制 ---------- */
    if (g_run)
    {
#if ENABLE_SPEED_LOOP
        /* ---- 速度闭环模式：速度环输出基础 PWM，转向环叠加差速 ---- */
        int32_t speed_outL, speed_outR;

        // 速度 PID 计算（目标速度 vs 实际编码器速度）
        speed_outL = pid_calc(&g_pid_speedL, (int32_t)g_speedL_now);
        speed_outR = pid_calc(&g_pid_speedR, (int32_t)g_speedR_now);

        // 合成：基础速度 ± 转向差速
        left_duty  = speed_outL - steer_diff;
        right_duty = speed_outR + steer_diff;
#else
        /* ---- 开环速度模式：固定基础速度 + 转向差速 ---- */
        left_duty  = BASE_SPEED - steer_diff;
        right_duty = BASE_SPEED + steer_diff;
#endif
        motor_set_speed(left_duty, right_duty);
    }
    else
    {
        motor_stop();
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    /* ---------- 平台初始化 ---------- */
    clock_init(SYSTEM_CLOCK_80M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化

    /* ---------- 外设初始化 ---------- */
    motor_init();                      // 电机驱动
    encoder_init();                    // 编码器
    gray_init();                       // 灰度传感器

    /* ---------- IPS200 液晶初始化 ---------- */
    ips200_init(IPS200_TYPE_SPI);      // 根据实际接线选择 SPI 模式
    ips200_set_dir(IPS200_DEFAULT_DISPLAY_DIR);    // 显示方向（竖屏或横屏）
    ips200_set_font(IPS200_8X16_FONT);             // 字体大小 8x16
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);  // 画笔黑色，背景白色

    /* ---------- 启动提示 ---------- */
    ips200_show_string(0, 0, "Gray PID Test");
    ips200_show_string(0, 20, "BaseSpeed:");
    ips200_show_int(80, 20, BASE_SPEED, 4);
    system_delay_ms(800);

    /* ---------- 初始化转向 PID（Q10 定点格式） ---------- */
    pid_init(&g_pid_steer, PID_MODE_POSITIONAL,
             STEER_KP_Q10, STEER_KI_Q10, STEER_KD_Q10,   // 系数已放大1024倍
             0,                                           // 目标偏差 = 0（线在中间）
             MAX_TURN_DIFF,                               // 输出限幅 ±500
             STEER_INTEGRAL_LIMIT,                        // 积分限幅
             1);                                          // 死区（偏差<1 不动作）

#if ENABLE_SPEED_LOOP
    /* ---------- 初始化速度 PID ---------- */
    pid_init(&g_pid_speedL, PID_MODE_POSITIONAL,
             SPEED_KP_Q10, SPEED_KI_Q10, SPEED_KD_Q10,
             TARGET_SPEED,                                // 目标速度
             SPEED_OUT_LIMIT,                             // 输出限幅
             SPEED_INTEGRAL_LIMIT,
             1);                                          // 死区
    pid_init(&g_pid_speedR, PID_MODE_POSITIONAL,
             SPEED_KP_Q10, SPEED_KI_Q10, SPEED_KD_Q10,
             TARGET_SPEED,
             SPEED_OUT_LIMIT,
             SPEED_INTEGRAL_LIMIT,
             1);
#endif

    /* ---------- 启动 PIT 定时器（每 10ms 触发控制回调） ---------- */
    pit_ms_init(PIT_TIM_G6, CONTROL_PERIOD_MS, pit_control_callback, NULL);

    /* ---------- 主循环（仅用于调试显示） ---------- */
    while (1)
    {
        ips200_display();               // 刷新 IPS200 屏幕
        system_delay_ms(80);            // 约 12.5Hz 刷新率，避免闪烁
    }
}