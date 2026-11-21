#ifndef __KEY_H
#define __KEY_H

#include "zf_common_headfile.h"

// ===== 引脚 =====
#define KEY_PIN                 A30

// ===== 消抖参数（10ms/次）=====
#define KEY_DOWN_DEBOUNCE       5       // 按下消抖 80ms
#define KEY_UP_DEBOUNCE         5       // 释放消抖 80ms
#define KEY_LONG_HOLD           150     // 长按 1.5s

typedef enum
{
    KEY_EVENT_NONE      = 0,
    KEY_EVENT_CLICK     = 1,
    KEY_EVENT_LONG      = 2,
} key_event_enum;

void            key_init2            (void);
void            key_scan            (void);
key_event_enum  key_get_event       (void);
void            key_clear_event     (void);

#endif