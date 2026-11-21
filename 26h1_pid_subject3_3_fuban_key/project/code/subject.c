/*
 * subject.c
 * 第三题状态机实现
 */

#include "zf_common_headfile.h"

/* 外部系统时间接口（在主程序中实现） */
extern uint32_t system_get_ms(void);

/* 内部静态变量 */
static subject3_state_t s_state = SUB3_IDLE;
static float s_target_pos = 0.0f;       // 当前目标位置 (cm)
static uint32_t s_state_enter_ms = 0;
static bool s_started = false;

subject3_state_t subject3_get_state(void)
{
    return s_state;
}

void subject3_init(void)
{
    s_state = SUB3_IDLE;
    s_target_pos = 0.0f;
    s_state_enter_ms = 0;
    s_started = false;
}

void subject3_start(void)
{
    if (s_state == SUB3_IDLE) {
        s_state = SUB3_GO_POS5;
        s_state_enter_ms = system_get_ms();
        s_started = true;
        s_target_pos = 3.5f;          // 立即设置初始目标，确保第一帧即可用
    }
}


float subject3_update(float measured_pos, float measured_speed, float dt)
{
    (void)dt;   // 状态机只看时间，不依赖 dt

    if (!s_started) {
        return 0.0f;  // 默认停在中心
    }

    uint32_t now = system_get_ms();
    float threshold = 2.0f;   // 判断到位的位置误差 (cm)

    switch (s_state)
    {
        case SUB3_GO_POS5:
            s_target_pos = 3.5f;
            if (fabsf(measured_pos - 5.0f) < threshold && fabsf(measured_speed) < 0.5f) {
                s_state = SUB3_WAIT_POS5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_WAIT_POS5:
            s_target_pos = 5.0f;
            if (now - s_state_enter_ms >= 100) {  // 稳定保持 500ms
                s_state = SUB3_GO_NEG5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_GO_NEG5:
            s_target_pos = -2.5f;
            if (fabsf(measured_pos + 5.0f) < threshold && fabsf(measured_speed) < 5.0f) {
                s_state = SUB3_WAIT_NEG5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_WAIT_NEG5:
            s_target_pos = -5.0f;
            if (now - s_state_enter_ms >= 1500) { // 稳定 1s 后任务完成
                s_state = SUB3_DONE;
            }
            break;

        case SUB3_DONE:
        case SUB3_IDLE:
        default:
            s_target_pos = -5.0f; // 完成后保持在 -5cm
            break;
    }

    return s_target_pos;
}