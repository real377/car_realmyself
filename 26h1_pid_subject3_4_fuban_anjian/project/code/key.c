#include "zf_common_headfile.h"

typedef enum { SM_IDLE, SM_DOWN_DEBOUNCE, SM_PRESSED, SM_UP_DEBOUNCE } key_sm_t;

typedef struct
{
    key_sm_t        sm;
    uint16_t        cnt;
    key_event_enum  event;
} key_slot_t;

static key_slot_t     keys[KEY_COUNT];
static const uint32_t pin_list[KEY_COUNT] = KEY_PIN_LIST;

void key_init2(void)
{
    for (int i = 0; i < KEY_COUNT; i++)
    {
        gpio_init(pin_list[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        keys[i].sm    = SM_IDLE;
        keys[i].cnt   = 0;
        keys[i].event = KEY_EVENT_NONE;
    }
}

void key_scan(void)
{
    for (int i = 0; i < KEY_COUNT; i++)
    {
        uint8_t pressed = (gpio_get_level(pin_list[i]) != GPIO_HIGH);
        key_slot_t *k = &keys[i];

        switch (k->sm)
        {
            case SM_IDLE:
                if (pressed) { k->sm = SM_DOWN_DEBOUNCE; k->cnt = 1; }
                break;

            case SM_DOWN_DEBOUNCE:
                if (pressed)
                {
                    k->cnt++;
                    if (k->cnt >= KEY_DOWN_DEBOUNCE) { k->sm = SM_PRESSED; k->cnt = 0; }
                }
                else { k->sm = SM_IDLE; k->cnt = 0; }
                break;

            case SM_PRESSED:
                if (pressed)
                {
                    k->cnt++;
                    if (k->cnt >= KEY_LONG_HOLD && k->event == KEY_EVENT_NONE)
                        k->event = KEY_EVENT_LONG;
                }
                else { k->sm = SM_UP_DEBOUNCE; k->cnt = 1; }
                break;

            case SM_UP_DEBOUNCE:
                if (!pressed)
                {
                    k->cnt++;
                    if (k->cnt >= KEY_UP_DEBOUNCE)
                    {
                        if (k->event != KEY_EVENT_LONG)
                            k->event = KEY_EVENT_CLICK;
                        k->sm  = SM_IDLE;
                        k->cnt = 0;
                    }
                }
                else { k->sm = SM_PRESSED; k->cnt = 0; }
                break;
        }
    }
}

key_event_enum key_get_event(my_key_enum idx)
{
    key_event_enum ret = keys[idx].event;
    keys[idx].event = KEY_EVENT_NONE;
    return ret;
}

void key_clear_event(my_key_enum idx)
{
    keys[idx].event = KEY_EVENT_NONE;
}

void key_clear_all(void)
{
    for (int i = 0; i < KEY_COUNT; i++)
        keys[i].event = KEY_EVENT_NONE;
}