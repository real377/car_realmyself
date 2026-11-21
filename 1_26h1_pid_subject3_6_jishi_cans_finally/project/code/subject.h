#ifndef __SUBJECT_H__
#define __SUBJECT_H__

#include <stdint.h>
#include <stdbool.h>

/* 任务状态（第三问使用） */
typedef enum {
    SUB3_IDLE,
    SUB3_GO_POS5,
    SUB3_WAIT_POS5,
    SUB3_GO_NEG5,
    SUB3_WAIT_NEG5,
    SUB3_DONE
} subject3_state_t;

/* 任务模式 */
typedef enum {
    TASK_MODE_3,      // 折返 +5/-5
    TASK_MODE_4_5,    // 保持中心0
    TASK_MODE_6       // 保持用户指定位置
} task_mode_t;

/* 原有第三问接口（维持不变） */
subject3_state_t subject3_get_state(void);
void            subject3_init(void);
void            subject3_start(void);
float           subject3_update(float measured_pos, float measured_speed, float dt);

/* 新增：多模式接口 */
void  subject_set_task_mode(task_mode_t mode, float user_target);
float subject_get_target(void);

#endif