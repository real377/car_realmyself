#include "zf_common_headfile.h"

extern uint32_t system_get_ms(void);

void stopwatch_init(stopwatch_t *sw)
{
    sw->state = SW_IDLE;
    sw->start_ms = 0;
    sw->elapsed_ms = 0;
}

void stopwatch_start(stopwatch_t *sw)
{
    if (sw->state == SW_IDLE) {
        sw->state = SW_RUNNING;
        sw->start_ms = system_get_ms();
        sw->elapsed_ms = 0;
    }
}

void stopwatch_stop(stopwatch_t *sw)
{
    if (sw->state == SW_RUNNING) {
        sw->elapsed_ms = system_get_ms() - sw->start_ms;
        sw->state = SW_STOPPED;
    }
}

uint32_t stopwatch_get_ms(const stopwatch_t *sw)
{
    if (sw->state == SW_RUNNING)
        return system_get_ms() - sw->start_ms;
    else if (sw->state == SW_STOPPED)
        return sw->elapsed_ms;
    return 0;
}

stopwatch_state_t stopwatch_get_state(const stopwatch_t *sw)
{
    return sw->state;
}