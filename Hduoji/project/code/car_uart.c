/*****************************************************************************
 * 文件名称: car_uart.c
 * 功能描述: 接收 K230 偏差+速度数据包，并缓存最近一帧原始字节供调试
 ****************************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"

car_data_t s_latest_data = {0, 0};
static uint8_t    s_new_data_flag = 0;

// 最近一帧的原始 5 字节（用于 OLED 显示）
static uint8_t s_raw_frame[5];
static volatile uint8_t s_raw_updated = 0;  // 原子标志，主循环中消费

// ===================== 环形缓冲区 =====================
typedef struct {
    uint8_t  buf[CAR_UART_RBUF_SIZE];
    volatile uint16_t head;
    uint16_t tail;
} ring_buf_t;

static ring_buf_t rbuf;

static void ring_buf_init(ring_buf_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    for (int i = 0; i < CAR_UART_RBUF_SIZE; i++)
        rb->buf[i] = 0;
}

static void ring_buf_put_isr(ring_buf_t *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % CAR_UART_RBUF_SIZE;
    if (next != rb->tail) {
        rb->buf[rb->head] = byte;
        rb->head = next;
    }
}

static uint8_t ring_buf_get(ring_buf_t *rb, uint8_t *out) {
    if (rb->tail == rb->head) return 0;
    *out = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % CAR_UART_RBUF_SIZE;
    return 1;
}

// ===================== 字节计数器 =====================
static volatile uint16_t total_bytes = 0;

uint16_t car_uart_byte_count(void) {
    uint16_t count;
    __disable_irq();
    count = total_bytes;
    __enable_irq();
    return count;
}

// ===================== UART 中断回调 =====================
static void rx_isr(uint32_t event, void *ptr) {
    (void)event;
    (void)ptr;
    uint8_t byte;
    while (uart_query_byte(CAR_UART_INDEX, &byte) == ZF_TRUE) {
        ring_buf_put_isr(&rbuf, byte);
        total_bytes++;
    }
}

// ===================== 初始化 =====================
void car_uart_init(void) {
    ring_buf_init(&rbuf);
    total_bytes = 0;
    s_new_data_flag = 0;

    uart_init(CAR_UART_INDEX, CAR_UART_BAUD,
              CAR_UART_TX_PIN, CAR_UART_RX_PIN);
    uart_set_callback(CAR_UART_INDEX, rx_isr, NULL);
	interrupt_set_priority (UART2_INT_IRQn, 0);
    uart_set_interrupt_config(CAR_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

// ===================== 帧解析状态机 =====================
typedef enum {
    WAIT_HEADER,
    WAIT_DEV_H,
    WAIT_DEV_L,
    WAIT_SPEED,
    WAIT_CHECKSUM
} parse_state_t;

static uint8_t parse_byte(uint8_t byte, car_data_t *out) {
    static parse_state_t state = WAIT_HEADER;
    static uint8_t raw[5];      // 暂存当前帧的原始字节
    static uint8_t raw_idx = 0;
    int16_t dev;

    switch (state) {
        case WAIT_HEADER:
            if (byte == 0xAA) {
                raw[0] = byte;
                raw_idx = 1;
                state = WAIT_DEV_H;
            }
            break;

        case WAIT_DEV_H:
            raw[raw_idx++] = byte;
            state = WAIT_DEV_L;
            break;

        case WAIT_DEV_L:
            raw[raw_idx++] = byte;
            state = WAIT_SPEED;
            break;

        case WAIT_SPEED:
            raw[raw_idx++] = byte;
            state = WAIT_CHECKSUM;
            break;

        case WAIT_CHECKSUM:
        {
            raw[raw_idx] = byte;
            uint8_t calc = 0xAA ^ raw[1] ^ raw[2] ^ raw[3];
            state = WAIT_HEADER;  // 复位
            if (byte == calc) {
                dev = ((int16_t)raw[1] << 8) | raw[2];
								// 在 parse_byte 的 WAIT_CHECKSUM 分支中，成功判定后：
								out->deviation = dev;
								out->speed = (int8_t)raw[3];   // 强制转为有符号
							// 保存完整帧到全局缓存（用于显示）
                for (uint8_t i=0; i<5; i++) {
                    s_raw_frame[i] = raw[i];
                }
                s_raw_updated = 1;
                return 1;
            }
            // 校验错误：丢弃该帧，但不清空 raw（无影响）
        } break;
    }
    return 0;
}

// ===================== 轮询 =====================
uint8_t car_uart_poll(car_data_t *out) {
    uint8_t byte;
    car_data_t temp;
    while (ring_buf_get(&rbuf, &byte)) {
        if (parse_byte(byte, &temp)) {
            s_latest_data = temp;
            s_new_data_flag = 1;
            if (out) *out = temp;
            return 1;
        }
    }
    return 0;
}

// ===================== 发送 =====================
void car_uart_send_byte(uint8_t byte) {
    uart_write_byte(CAR_UART_INDEX, byte);
}

void car_uart_send_str(const char *str) {
    uart_write_string(CAR_UART_INDEX, str);
}

// ===================== 数据查询 =====================
uint8_t car_uart_is_new_data(void) { return s_new_data_flag; }

car_data_t car_uart_get_data(void) { return s_latest_data; }

void car_uart_clear_flag(void) { s_new_data_flag = 0; }

// ===================== 调试函数：读取最近一帧原始字节 =====================
uint8_t car_uart_get_raw_frame(uint8_t buf[5]) {
    if (s_raw_updated) {
        __disable_irq();
        for (int i=0; i<5; i++) buf[i] = s_raw_frame[i];
        s_raw_updated = 0;
        __enable_irq();
        return 1;
    }
    return 0;
}