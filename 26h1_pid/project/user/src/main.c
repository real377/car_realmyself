/******************************************************************
 * 文件名称          main.c
 * 功能描述          球平衡速度环调试（带数据保护）
 *                   串口获取钢球速度，PID计算输出控制舵机
 * 依赖             pid.c/h, car_uart.c/h, OLED.c/h, 逐飞基础库
 ******************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"
#include "OLED.h"
#include "pid.h"

/* ========== 硬件定义（请确认与实际连线一致） ========== */
#define BALL_SERVO_PIN      SERVO_2
#define BALL_SERVO_MID      SERVO2_MID       // 4900
#define PWM_OUT_LIMIT       800              // PWM 偏移量限幅
#define CONTROL_PERIOD_S    0.005f           // 5ms 控制周期

/* ========== 全局变量 ========== */
car_data_t k230_data;                    // 最新传感器数据
pid_t speed_pid;                         // 速度环 PID 控制器
float speed_output = 0.0f;               // PID 输出量（观察用）
volatile uint8_t new_data_ready = 0;     // 新数据标志

/* 函数声明 */
void pit_handler_1(uint32 state, void *ptr); // 控制中断（5ms）
void pit_handler_2(uint32 state, void *ptr); // 串口轮询中断（5ms）

/* 将整数转换为字符串（OLED 显示用） */
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

    // 如果 PWM 尚未初始化，请确保在 Car_Init() 中或手动初始化
    // pwm_init(BALL_SERVO_PIN, 300, BALL_SERVO_MID);

	  duoji_init();
	
	
    /* 初始化速度 PID (参数需要实验整定) */
    pid_init(&speed_pid, 20.0f, 0.00f, 0.01f, 10.0f, 500.0f);

	
	
    /* 中断优先级 */
    interrupt_set_priority(TIMA0_INT_IRQn, 2);   // 控制中断优先级高
    interrupt_set_priority(TIMA1_INT_IRQn, 1);   // 轮询中断
    pit_ms_init(PIT_TIM_A0, 1, pit_handler_1, NULL);
    pit_ms_init(PIT_TIM_A1, 5, pit_handler_2, NULL);

    char str[17];

    while (1)
    {
        /* ---- OLED 显示（每循环更新） ---- */
        int_to_str(k230_data.deviation, str, 6);
        OLED_ShowString(1, 1, "D:");
        OLED_ShowString(1, 3, str);

        int_to_str(k230_data.speed, str, 4);
        OLED_ShowString(2, 1, "S:");
        OLED_ShowString(2, 3, str);

        int_to_str((int)speed_output, str, 5);
        OLED_ShowString(3, 1, "Out:");
        OLED_ShowString(3, 5, str);

        int_to_str(0, str, 4);
        OLED_ShowString(4, 1, "Tgt:   0");
    }
}

/* ==============================================================
 *  pit_handler_1 (TIMA0, 5ms) —— 控制主循环
 *  功能：当有新数据且数据合理时，执行速度环 PID
 * ============================================================== */
void pit_handler_1(uint32 state, void *ptr)
{
    (void)state; (void)ptr;

    if (new_data_ready)
    {
        new_data_ready = 0;     // 消费标志

        /* 获取最新数据 */
        k230_data = car_uart_get_data();

        /* ---------- 数据合理性检查 ---------- */
        // 偏差绝对值不能超过摆杆物理极限 (半杆长约12.5cm => 1250 单位0.01cm)
        if (abs(k230_data.deviation) > 1500)
        {
            // 数据异常，忽略本帧，舵机返回中位并重置 PID 状态（可选）
            pid_init(&speed_pid, speed_pid.kp, speed_pid.ki, speed_pid.kd,
                     speed_pid.integral_limit, speed_pid.output_limit);
            pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID);
            return;
        }

        // 速度合理性检查：钢球在重力摆杆上基本不会超过 100 cm/s
        if (abs(k230_data.speed) > 100)
        {
            // 速度异常，也忽略并回中
            pid_init(&speed_pid, speed_pid.kp, speed_pid.ki, speed_pid.kd,
                     speed_pid.integral_limit, speed_pid.output_limit);
            pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID);
            return;
        }
        /* ----------------------------------- */

        /* 设定目标速度（当前固定为 0，可按键修改） */
        float target_speed = 0.0f;

        /* 测量速度（cm/s） */
        float measured_speed = (float)k230_data.speed;

        /* PID 计算 */
        float output = pid_update(&speed_pid, target_speed, measured_speed, CONTROL_PERIOD_S);
        speed_output = output;

        /* 输出到舵机 */
        int16 pwm_offset = (int16)output;
        pwm_offset = func_limit_ab(pwm_offset, -PWM_OUT_LIMIT, PWM_OUT_LIMIT);
        pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID - pwm_offset);  // 方向可能需要调换
    }
}

/* ==============================================================
 *  pit_handler_2 (TIMA1, 5ms) —— 串口数据轮询
 * ============================================================== */
void pit_handler_2(uint32 state, void *ptr)
{
    (void)state; (void)ptr;

    if (car_uart_poll(NULL) != 0)    // 解析出一帧有效数据
    {
        new_data_ready = 1;          // 通知控制中断
    }
}