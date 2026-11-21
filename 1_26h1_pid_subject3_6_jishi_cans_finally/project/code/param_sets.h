#ifndef __PARAM_SETS_H__
#define __PARAM_SETS_H__

#include "zf_common_headfile.h"

typedef struct {
    float pos_kp, pos_ki, pos_kd;
    float pos_integral_limit, pos_output_limit;
    float speed_kp, speed_ki, speed_kd;
    float speed_integral_limit, speed_output_limit;
    float speed_deadzone;
    float max_speed_accel;
    uint8_t enable_pos_loop;
} param_set_t;

/* 参数集：索引 0 用于第三问，索引 1 用于第四、五、六问 */
static const param_set_t param_table[] = {
    // [0] 第三问：静止折返
    { 5.0f, 0.02f, 0.0f, 3.0f, 20.0f,
      18.0f, 0.0f, 0.01f, 10.0f, 200.0f,
      1.0f, 5.0f, 1 },
    // [1] 第四、五、六问：小车运动，抗扰更强
    { 7.0f, 0.1f, 0.0f, 3.0f, 0.0f,
      40.0f, 0.0f, 0.01f, 10.0f, 280.0f,
      0.0f, 60.0f, 0 },
};

#endif