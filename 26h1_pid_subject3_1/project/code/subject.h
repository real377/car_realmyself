/*
 * subject.h
 * 第三题状态机模块
 */

#ifndef __SUBJECT_H__
#define __SUBJECT_H__

#include <stdint.h>
#include <stdbool.h>

/* 任务状态 */
typedef enum {
    SUB3_IDLE,
    SUB3_GO_POS5,
    SUB3_WAIT_POS5,
    SUB3_GO_NEG5,
    SUB3_WAIT_NEG5,
    SUB3_DONE
} subject3_state_t;

/* 获取当前状态（用于 OLED 显示） */
subject3_state_t subject3_get_state(void);

/*
 * 初始化状态机，回到 IDLE 并重置目标
 */
void subject3_init(void);

/*
 * 开始任务（例如按键触发）
 */
void subject3_start(void);

/*
 * 每控制周期调用一次，传入当前实际位置和速度，返回该周期的目标位置（cm）
 * 参数：
 *   measured_pos  : 当前球位置 (cm)
 *   measured_speed: 当前球速度 (cm/s)
 *   dt            : 控制周期 (秒)
 * 返回值：目标位置 (cm)，由外部位置环使用
 */
float subject3_update(float measured_pos, float measured_speed, float dt);

#endif /* __SUBJECT_H__ */