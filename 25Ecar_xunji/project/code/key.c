#include "zf_common_headfile.h"

typedef enum { SM_IDLE, SM_DOWN_DEBOUNCE, SM_PRESSED, SM_UP_DEBOUNCE } key_sm_t;

static key_sm_t       sm     = SM_IDLE;
static uint16_t       cnt    = 0;
static key_event_enum event  = KEY_EVENT_NONE;

void key_init2(void)
{
    gpio_init(KEY_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    sm    = SM_IDLE;
    cnt   = 0;
    event = KEY_EVENT_NONE;
}

void key_scan(void)
{
    uint8_t pressed = (gpio_get_level(KEY_PIN) != GPIO_HIGH);

    switch (sm)
    {
        case SM_IDLE:
            if (pressed) { sm = SM_DOWN_DEBOUNCE; cnt = 1; }
            break;

        case SM_DOWN_DEBOUNCE:
            if (pressed)
            {
                cnt++;
                if (cnt >= KEY_DOWN_DEBOUNCE) { sm = SM_PRESSED; cnt = 0; }
            }
            else { sm = SM_IDLE; cnt = 0; }
            break;

        case SM_PRESSED:
            if (pressed)
            {
                cnt++;
                if (cnt >= KEY_LONG_HOLD && event == KEY_EVENT_NONE)
                    event = KEY_EVENT_LONG;
            }
            else { sm = SM_UP_DEBOUNCE; cnt = 1; }
            break;

        case SM_UP_DEBOUNCE:
            if (!pressed)
            {
                cnt++;
                if (cnt >= KEY_UP_DEBOUNCE)
                {
                    if (event != KEY_EVENT_LONG)
                        event = KEY_EVENT_CLICK;
                    sm  = SM_IDLE;
                    cnt = 0;
                }
            }
            else { sm = SM_PRESSED; cnt = 0; }
            break;
    }
}

key_event_enum key_get_event(void)
{
    key_event_enum ret = event;
    event = KEY_EVENT_NONE;
    return ret;
}

void key_clear_event(void)
{
    event = KEY_EVENT_NONE;
}