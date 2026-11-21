/******************************************************************
 * 文件名称          main.c
 * 功能描述          球平衡串级控制（位置外环 + 速度内环）
 *                   视觉反馈 → 位置PID → 目标速度 → 速度PID → 舵机
 * 依赖             pid.c/h, car_uart.c/h, OLED.c/h, 逐飞基础库
 ******************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"
#include "OLED.h"
#include "pid.h"

/* ========== 硬件定义 ========== */
#define BALL_SERVO_PIN      SERVO_2
#define BALL_SERVO_MID      SERVO2_MID          // 4900
#define PWM_OUT_LIMIT       800                 // 最终 PWM 偏移限幅
#define CONTROL_PERIOD_S    0.005f              // 控制周期 5ms

/* ========== 全局变量 ========== */
car_data_t k230_data;
pid_t pos_pid;                                 // 位置环（外环）
pid_t speed_pid;                               // 速度环（内环）
float pos_output = 0.0f;                       // 位置环输出（目标速度，观察用）
float speed_output = 0.0f;                     // 速度环输出（观察用）
volatile uint8_t new_data_ready = 0;


int a = 0;

/* 函数声明 */
void pit_handler_1(uint32 state, void *ptr);
void pit_handler_2(uint32 state, void *ptr);

/* 整数转字符串（OLED 显示用） */
static void int_to_str(int num, char *buf, int width) {
    int i = 0, j, negative = 0;
    char temp[7];
    if (num < 0) { negative = 1; num = -num; }
    do { temp[i++] = (num % 10) + '0'; num /= 10; } while (num && i < 6);
    if (negative) temp[i++] = '-';
    while (i < width) temp[i++] = ' ';
    for (j = 0; j < i; j++) buf[j] = temp[i - j - 1];
    buf[i] = '\0';
}

int main(void)
{
    /* 系统初始化 */
    clock_init(SYSTEM_CLOCK_80M);
    OLED_Init();
    car_uart_init();


    duoji_init();   // 如果已移除 duoji 模块，请注释掉

    /* ---------- 初始化 PID ---------- */

    // 位置环参数（需要实验整定，初始值如下）
    pid_init(&pos_pid,
        5.0f,           // Kp: 位置误差比例增益
        0.1f,          // Ki: 积分，消除稳态位置误差
        0.0f,          // Kd: 微分，抑制位置过冲
        3.0f,           // 积分饱和上限 (cm)
        30.0f);         // 输出限幅：最大目标速度 (cm/s)

    // 速度环参数（沿用你已调好的值，不要随便改动）
    pid_init(&speed_pid,
        20.0f,          // Kp
        0.00f,          // Ki
        0.01f,          // Kd
        10.0f,          // 积分上限
        500.0f);        // 输出限幅（原始输出，会再经过 PWM_OUT_LIMIT）

    /* 中断优先级 */
    interrupt_set_priority(TIMA0_INT_IRQn, 2);   // 控制中断
    interrupt_set_priority(TIMA1_INT_IRQn, 1);   // 轮询中断
    pit_ms_init(PIT_TIM_A0, 1, pit_handler_1, NULL);   // 1ms 计数基
    pit_ms_init(PIT_TIM_A1, 5, pit_handler_2, NULL);   // 5ms 轮询

    char str[17];

    while (1)
    {
        /* ---- OLED 显示 ---- */
        // 第1行：偏差 (0.01cm) 或位置 (cm)
        int_to_str(k230_data.deviation, str, 6);
        OLED_ShowString(1, 1, "Dev:");
        OLED_ShowString(1, 5, str);

        // 第2行：速度 (cm/s)
        int_to_str(k230_data.speed, str, 4);
        OLED_ShowString(2, 1, "Spd:");
        OLED_ShowString(2, 5, str);

        // 第3行：位置环输出 (目标速度)
        int_to_str((int)pos_output, str, 4);
        OLED_ShowString(3, 1, "Tsp:");
        OLED_ShowString(3, 5, str);

        // 第4行：速度环输出 (最终 PWM 偏移)
        int_to_str((int)speed_output, str, 5);
        OLED_ShowString(4, 1, "Out:");
        OLED_ShowString(4, 5, str);
    }
}

/* ==============================================================
 *  pit_handler_1 (被 TIMA0 以 1ms 周期调用)
 *  我们在此实现 5ms 分频的控制计算
 * ============================================================== */
void pit_handler_1(uint32 state, void *ptr)
{
    (void)state; (void)ptr;
    static uint8_t tick = 0;
		
    tick++;
    if (tick < 5) return;          // 5ms 执行一次
	
		a++;
    tick = 0;

    if (new_data_ready)
    {
        new_data_ready = 0;

        k230_data = car_uart_get_data();

        /* ---------- 数据合理性检查（与之前相同） ---------- */
        if (abs(k230_data.deviation) > 1500)
        {
            // 异常数据，复位 PID 并回中
            pid_init(&pos_pid, pos_pid.kp, pos_pid.ki, pos_pid.kd,
                     pos_pid.integral_limit, pos_pid.output_limit);
            pid_init(&speed_pid, speed_pid.kp, speed_pid.ki, speed_pid.kd,
                     speed_pid.integral_limit, speed_pid.output_limit);
            pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID);
            speed_output = 0.0f;
            pos_output = 0.0f;
            return;
        }
        if (abs(k230_data.speed) > 100)
        {
            pid_init(&pos_pid, pos_pid.kp, pos_pid.ki, pos_pid.kd,
                     pos_pid.integral_limit, pos_pid.output_limit);
            pid_init(&speed_pid, speed_pid.kp, speed_pid.ki, speed_pid.kd,
                     speed_pid.integral_limit, speed_pid.output_limit);
            pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID);
            speed_output = 0.0f;
            pos_output = 0.0f;
            return;
        }
        /* ------------------------------------------------ */

        /* ---- 位置外环（cm 制） ---- */
        float target_pos = 0.0f;                           // 目标位置：摆杆中心
        float measured_pos = k230_data.deviation * 0.01f;  // 转换为 cm

        // 位置环输出 = 目标速度
        float target_speed = pid_update(&pos_pid, target_pos, measured_pos, CONTROL_PERIOD_S);
        pos_output = target_speed;

        /* ---- 速度内环 ---- */
        float measured_speed = (float)k230_data.speed;
        float output = pid_update(&speed_pid, target_speed, measured_speed, CONTROL_PERIOD_S);
        speed_output = output;

        /* ---- 输出到舵机 ---- */
        int16 pwm_offset = (int16)output;
        pwm_offset = func_limit_ab(pwm_offset, -PWM_OUT_LIMIT, PWM_OUT_LIMIT);\
				
				
        pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID - pwm_offset);



    }
}

/* ==============================================================
 *  pit_handler_2 (TIMA1, 5ms) —— 串口数据轮询
 * ============================================================== */
void pit_handler_2(uint32 state, void *ptr)
{
    (void)state; (void)ptr;
    if (car_uart_poll(NULL) != 0)
    {
        new_data_ready = 1;
    }
}