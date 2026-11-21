#ifndef __KEY_H
#define __KEY_H

#include "zf_common_headfile.h"

// ===== 按键引脚列表（仅 B21）=====
#define KEY_PIN_LIST        { B21 }
#define KEY_COUNT           1

// ===== 消抖参数（10ms/tick）=====
#define KEY_DOWN_DEBOUNCE   5
#define KEY_UP_DEBOUNCE     5
#define KEY_LONG_HOLD       150

// ===== 按键索引（只用一个按键 B21）=====
typedef enum
{
    MY_KEY_B21 = 0,
} my_key_enum;

// ===== 事件类型 =====
typedef enum
{
    KEY_EVENT_NONE  = 0,
    KEY_EVENT_CLICK = 1,
    KEY_EVENT_LONG  = 2,
} key_event_enum;

void            key_init2            (void);
void            key_scan            (void);
key_event_enum  key_get_event       (my_key_enum idx);
void            key_clear_event     (my_key_enum idx);
void            key_clear_all       (void);

#endif