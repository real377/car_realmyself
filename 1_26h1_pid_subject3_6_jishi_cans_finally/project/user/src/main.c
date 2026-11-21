/******************************************************************
 * 文件名称          main.c
 * 功能描述          球平衡串级控制 + 第三/四/五/六问 + 计时
 ******************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"
#include "OLED.h"
#include "pid.h"
#include "key.h"
#include "subject.h"
#include "stopwatch.h"
#include "param_sets.h"

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

volatile uint32_t g_sys_ms = 0;
uint8_t enable_pos_loop = 1;

stopwatch_t task_timer;
const param_set_t *cur_param = &param_table[0];   // 默认第三问

/* 模式选择变量 */
static int selected_task = 0;   // 0:Task3, 1:Task4, 2:Task5, 3:Task6
static int mode_locked = 0;
static float user_target = 0.0f;
static int target_set = 0;

void pit_handler_1(uint32 state, void *ptr);
void pit_handler_2(uint32 state, void *ptr);

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

uint32_t system_get_ms(void) {
    return g_sys_ms;
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    OLED_Init();
    car_uart_init();
    duoji_init();
    key_init2();

    pid_init(&pos_pid, 5.0f, 0.02f, 0.0f, 3.0f, 20.0f);
    pid_init(&speed_pid, 18.0f, 0.0f, 0.01f, 10.0f, 200.0f);

    subject3_init();
    stopwatch_init(&task_timer);

    interrupt_set_priority(TIMA0_INT_IRQn, 2);
    interrupt_set_priority(TIMA1_INT_IRQn, 1);
    pit_ms_init(PIT_TIM_A0, 1, pit_handler_1, NULL);
    pit_ms_init(PIT_TIM_A1, 5, pit_handler_2, NULL);

    char str[17];
    char time_buf[12];

    while (1) {
        /* ========== 模式选择（未锁定） ========== */
        if (!mode_locked) {
            if (key_get_event(MY_KEY_A31) == KEY_EVENT_CLICK) {
                selected_task = (selected_task + 1) % 4;
            }
            if (key_get_event(MY_KEY_A30) == KEY_EVENT_CLICK) {
                mode_locked = 1;
                if (selected_task == 0) {
                    cur_param = &param_table[0];
                    subject3_init();                      // 第三问原版
                } else {
                    cur_param = &param_table[1];
                    if (selected_task == 3) {
                        user_target = 0.0f;
                        target_set = 0;
                    } else {
                        subject_set_task_mode(TASK_MODE_4_5, 0.0f);
                    }
                }
            }

            sprintf(str, "Task %d", selected_task + 3);
            OLED_ShowString(1, 1, str);
            OLED_ShowString(2, 1, "K2:Sw K1:Lock");
            system_delay_ms(50);
            continue;
        }

        /* ========== 第六问目标设定 ========== */
        if (selected_task == 3 && !target_set && mode_locked) {
            if (key_get_event(MY_KEY_A31) == KEY_EVENT_CLICK) user_target += 0.5f;
            if (key_get_event(MY_KEY_B0)  == KEY_EVENT_CLICK) user_target -= 0.5f;
            if (key_get_event(MY_KEY_A30) == KEY_EVENT_CLICK) {
                target_set = 1;
                subject_set_task_mode(TASK_MODE_6, user_target);
            }
            sprintf(str, "Target:%.1f", user_target);
            OLED_ShowString(1, 1, str);
            OLED_ShowString(2, 1, "K2:+0.5 K3:-0.5");
            OLED_ShowString(3, 1, "K1:Confirm");
            system_delay_ms(50);
            continue;
        }

        /* ========== 启动/停止任务 ========== */
        if (selected_task == 0 || selected_task == 1 || selected_task == 2 ||
            (selected_task == 3 && target_set)) {
            if (key_get_event(MY_KEY_A30) == KEY_EVENT_CLICK) {
                if (subject3_get_state() == SUB3_IDLE) {
                    subject3_start();                  // 第三问直接调用原版
                    stopwatch_start(&task_timer);
                }
            }
            if (selected_task != 0 && key_get_event(MY_KEY_B0) == KEY_EVENT_CLICK) {
                if (stopwatch_get_state(&task_timer) == SW_RUNNING)
                    stopwatch_stop(&task_timer);
            }
            if (selected_task == 0 && subject3_get_state() == SUB3_DONE &&
                stopwatch_get_state(&task_timer) == SW_RUNNING) {
                stopwatch_stop(&task_timer);
            }
        }

        /* ========== OLED 显示 ========== */
        sprintf(str, "T%d S:%d", selected_task + 3, subject3_get_state());
        OLED_ShowString(1, 1, str);

        int_to_str(k230_data.speed, str, 4);
        OLED_ShowString(2, 1, "Spd:");
        OLED_ShowString(2, 5, str);

        int_to_str((int)(subject_get_target() * 10), str, 4);
        OLED_ShowString(3, 1, "Tar(mm):");
        OLED_ShowString(3, 9, str);


        uint32_t elapsed = stopwatch_get_ms(&task_timer);
        uint32_t sec = elapsed / 1000;
        uint32_t ms  = elapsed % 1000;
        sprintf(time_buf, "%2u.%03u", sec, ms);
        OLED_ShowString(4, 1, "T:");
        OLED_ShowString(4, 3, time_buf);
    }
}

void pit_handler_1(uint32 state, void *ptr) {
    (void)state; (void)ptr;
    static uint8_t tick = 0;
    g_sys_ms++;
    key_scan();
    tick++;
    if (tick < 5) return;
    tick = 0;

    if (new_data_ready) {
        new_data_ready = 0;
        k230_data = car_uart_get_data();

        if (abs(k230_data.deviation) > 1500 || abs(k230_data.speed) > 100) {
            pid_init(&pos_pid, cur_param->pos_kp, cur_param->pos_ki, cur_param->pos_kd,
                     cur_param->pos_integral_limit, cur_param->pos_output_limit);
            pid_init(&speed_pid, cur_param->speed_kp, cur_param->speed_ki, cur_param->speed_kd,
                     cur_param->speed_integral_limit, cur_param->speed_output_limit);
            pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID);
            speed_output = 0.0f; pos_output = 0.0f;
            return;
        }

        float measured_pos = k230_data.deviation * 0.01f;
        float measured_speed = (float)k230_data.speed;

        float target_pos = subject3_update(measured_pos, measured_speed, CONTROL_PERIOD_S);

        float raw_target_speed = pid_update(&pos_pid, target_pos, measured_pos, CONTROL_PERIOD_S);

				
				
					
        #define MAX_SPEED_ACCEL   5.0f
        static float prev_target_speed = 0.0f;
        float delta = raw_target_speed - prev_target_speed;
        if (delta >  cur_param->max_speed_accel)  delta =  cur_param->max_speed_accel;
        if (delta < -cur_param->max_speed_accel)  delta = -cur_param->max_speed_accel;
        float target_speed = prev_target_speed + delta;
        prev_target_speed = target_speed;
        pos_output = target_speed;

        float output = pid_update(&speed_pid, target_speed, measured_speed, CONTROL_PERIOD_S);

        if (fabsf(target_speed - measured_speed) < cur_param->speed_deadzone) {
            output = 0.0f;
            speed_pid.integral = 0.0f;
        }

        speed_output = output;
        int16 pwm_offset = (int16)output;
        pwm_offset = func_limit_ab(pwm_offset, -PWM_OUT_LIMIT, PWM_OUT_LIMIT);
        pwm_set_duty(BALL_SERVO_PIN, BALL_SERVO_MID - pwm_offset);
    }
}

void pit_handler_2(uint32 state, void *ptr) {
    (void)state; (void)ptr;
    if (car_uart_poll(NULL) != 0)
        new_data_ready = 1;
}