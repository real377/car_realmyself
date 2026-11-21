#ifndef __STOPWATCH_H__
#define __STOPWATCH_H__
#include "zf_common_headfile.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SW_IDLE,
    SW_RUNNING,
    SW_STOPPED
} stopwatch_state_t;

typedef struct {
    stopwatch_state_t state;
    uint32_t start_ms;
    uint32_t elapsed_ms;
} stopwatch_t;

void stopwatch_init(stopwatch_t *sw);
void stopwatch_start(stopwatch_t *sw);
void stopwatch_stop(stopwatch_t *sw);
uint32_t stopwatch_get_ms(const stopwatch_t *sw);
stopwatch_state_t stopwatch_get_state(const stopwatch_t *sw);

#endif