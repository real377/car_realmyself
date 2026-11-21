/******************************************************************
 * 文件名称          main.c
 * 功能描述          球平衡串级控制 + 第三题状态机
 *                   可关闭位置环单独调试速度环
 ******************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"
#include "OLED.h"
#include "pid.h"
#include "subject.h"

/* ========== 硬件定义 ========== */
#define BALL_SERVO_PIN      SERVO_2
#define BALL_SERVO_MID      SERVO2_MID
#define PWM_OUT_LIMIT       800
#define CONTROL_PERIOD_S    0.005f

/* ========== 全局变量 ========== */
car_data_t k230_data;
pid_t pos_pid;
pid_t speed_pid;
float pos_output = 0.0f;
float speed_output = 0.0f;
volatile uint8_t new_data_ready = 0;

/* 系统时间（ms），在 1ms 中断中自增 */
volatile uint32_t g_sys_ms = 0;

/* 位置环使能标志：0 = 只使用速度环（目标速度=0），1 = 位置+速度串级 */
uint8_t enable_pos_loop = 0;  // 初始关闭，用按键或手动改

/* 函数声明 */
void pit_handler_1(uint32 state, void *ptr);
void pit_handler_2(uint32 state, void *ptr);

/* 整数转字符串 */
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

/* 为 subject 模块提供系统时间（ms） */
uint32_t system_get_ms(void)
{
    return g_sys_ms;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    OLED_Init();
    car_uart_init();

    // 初始化 PID
    pid_init(&pos_pid, 0.8f, 0.02f, 0.0f, 3.0f, 10.0f);
    pid_init(&speed_pid, 18.0f, 0.0f, 0.01f, 10.0f, 300.0f);

	  duoji_init();
	
	
    // 初始化第三题状态机
    subject3_init();

    // 中断配置
    interrupt_set_priority(TIMA0_INT_IRQn, 2);
    interrupt_set_priority(TIMA1_INT_IRQn, 1);
    pit_ms_init(PIT_TIM_A0, 1, pit_handler_1, NULL);
    pit_ms_init(PIT_TIM_A1, 5, pit_handler_2, NULL);

    char str[17];

    while (1)
    {
        // 第1行：偏差
        int_to_str(k230_data.deviation, str, 6);
        OLED_ShowString(1, 1, "Dev:");
        OLED_ShowString(1, 5, str);

        // 第2行：速度
        int_to_str(k230_data.speed, str, 4);
        OLED_ShowString(2, 1, "Spd:");
        OLED_ShowString(2, 5, str);

        // 第3行：目标速度 / 位置环输出
        if (enable_pos_loop) {
            int_to_str((int)pos_output, str, 4);
            OLED_ShowString(3, 1, "Tsp:");
        } else {
            OLED_ShowString(3, 1, "Tsp:   0   ");
        }
        OLED_ShowString(3, 5, str);

        // 第4行：最终输出
        int_to_str((int)speed_output, str, 5);
        OLED_ShowString(4, 1, "Out:");
        OLED_ShowString(4, 5, str);
/*
        // 第5行：任务状态
        subject3_state_t st = subject3_get_state();
        const char *state_str[] = {"Idle","Go+5","+5OK","Go-5","-5OK","Done"};
        OLED_ShowString(5, 1, state_str[st]);
				*/
    }
}

void pit_handler_1(uint32 state, void *ptr)
{
    (void)state; (void)ptr;
    static uint8_t tick = 0;

    // 系统时钟累加（1ms）
    g_sys_ms++;

    tick++;
    if (tick < 5) return;
    tick = 0;

    if (new_data_ready)
    {
        new_data_ready = 0;
        k230_data = car_uart_get_data();

        // 数据合理性检查
        if (abs(k230_data.deviation) > 1500 || abs(k230_data.speed) > 100)
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

        float measured_pos = k230_data.deviation * 0.01f;
        float measured_speed = (float)k230_data.speed;

        float target_pos;
        if (enable_pos_loop)
            target_pos = subject3_update(measured_pos, measured_speed, CONTROL_PERIOD_S);
        else
            target_pos = 0.0f;

        float target_speed = pid_update(&pos_pid, target_pos, measured_pos, CONTROL_PERIOD_S);
        pos_output = target_speed;

        float output = pid_update(&speed_pid, target_speed, measured_speed, CONTROL_PERIOD_S);

        // 方案一：死区
        #define SPEED_DEADZONE  1.0f
        if (fabsf(target_speed - measured_speed) < SPEED_DEADZONE) {
            output = 0.0f;
            speed_pid.integral = 0.0f;
        }

        speed_output = output;

        int16 pwm_offset = (int16)output;
        pwm_offset = func_limit_ab(pwm_offset, -PWM_OUT_LIMIT, PWM_OUT_LIMIT);
        pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID - pwm_offset);
				
				
				
    }
}

void pit_handler_2(uint32 state, void *ptr)
{
    (void)state; (void)ptr;
    if (car_uart_poll(NULL) != 0)
    {
        new_data_ready = 1;
    }
}