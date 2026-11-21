#include "subject.h"
#include "zf_common_headfile.h"

extern uint32_t system_get_ms(void);

/* ===== 内部变量（保留您原版的全部静态变量） ===== */
static subject3_state_t s_state = SUB3_IDLE;
static float s_target_pos = 0.0f;
static uint32_t s_state_enter_ms = 0;
static bool s_started = false;

/* 新增多模式变量 */
static task_mode_t current_mode = TASK_MODE_3;
static float user_hold_pos = 0.0f;   // 第六问目标

/* -------- 原有第三问接口（一字未改）-------- */
subject3_state_t subject3_get_state(void) {
    return s_state;
}

void subject3_init(void) {
    s_state = SUB3_IDLE;
    s_target_pos = 0.0f;
    s_state_enter_ms = 0;
    s_started = false;
    current_mode = TASK_MODE_3;    // 默认第三问
}

void subject3_start(void) {
    if (s_state == SUB3_IDLE) {
        s_state = SUB3_GO_POS5;
        s_state_enter_ms = system_get_ms();
        s_started = true;
    }
}

float subject3_update(float measured_pos, float measured_speed, float dt) {
    (void)dt;
    if (!s_started) return 0.0f;

    /* 如果是第四/五/六问，直接返回固定目标，不走状态机 */
    if (current_mode != TASK_MODE_3) {
        s_target_pos = (current_mode == TASK_MODE_6) ? user_hold_pos : 0.0f;
        return s_target_pos;
    }

    /* ---------- 第三问原始状态机 ---------- */
    uint32_t now = system_get_ms();
    float threshold = 2.0f;

    switch (s_state) {
        case SUB3_GO_POS5:
            s_target_pos = 4.0f;
            if (fabsf(measured_pos - 6.5f) < threshold && fabsf(measured_speed) < 0.5f) {
                s_state = SUB3_WAIT_POS5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_WAIT_POS5:
            s_target_pos = 5.0f;
            if (now - s_state_enter_ms >= 100) {
                s_state = SUB3_GO_NEG5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_GO_NEG5:
            s_target_pos = -1.2f;
            if (fabsf(measured_pos + 5.0f) < threshold && fabsf(measured_speed) < 5.0f) {
                s_state = SUB3_WAIT_NEG5;
                s_state_enter_ms = now;
            }
            break;

        case SUB3_WAIT_NEG5:
            s_target_pos = -5.0f;
            if (now - s_state_enter_ms >= 1500) {
                s_state = SUB3_DONE;
            }
            break;

        case SUB3_DONE:
        case SUB3_IDLE:
        default:
            s_target_pos = -5.0f;
            break;
    }
    return s_target_pos;
}

/* -------- 新增多模式接口 -------- */
void subject_set_task_mode(task_mode_t mode, float user_target) {
    current_mode = mode;
    if (mode == TASK_MODE_6) {
        user_hold_pos = user_target;
    }
    // 重置状态，确保模式切换后可以重新启动
    subject3_init();    // 调用原初始化，内部会将current_mode设回TASK_MODE_3，需修正
    current_mode = mode;   // 重新设置模式，因为subject3_init会重置为TASK_MODE_3
    if (mode == TASK_MODE_6) user_hold_pos = user_target;
}

float subject_get_target(void) {
    return s_target_pos;
}