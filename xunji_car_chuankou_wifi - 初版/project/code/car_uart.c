/*****************************************************************************
 * 文件名称: car_uart.c
 * 功能描述: 基于逐飞库的 UART 接收，解析 K230 单字节指令
 ****************************************************************************/
#include "zf_common_headfile.h"

// 新增内部状态
static uint8_t s_locked_id = 0;      // 锁定房间号
static uint8_t s_turn_dir = 0;       // 0: 无效/直行, 1: 左, 2: 右


// ===================== 环形缓冲区 =====================
typedef struct {
    uint8_t  buf[CAR_UART_RBUF_SIZE];
    volatile uint16_t head;
    uint16_t tail;
} ring_buf_t;

static ring_buf_t rbuf;

// 状态机变量（添加到 car_uart.c 文件顶部）
typedef enum {
    WAIT_HEADER,
    RECEIVE_CMD
} parse_state_t;
static parse_state_t pstate = WAIT_HEADER;

static uint8_t parse_byte_with_header(uint8_t byte, car_cmd_t *cmd) {
    switch (pstate) {
        case WAIT_HEADER:
            if (byte == 0xAA) {
                pstate = RECEIVE_CMD;
            }
            return 0; // 尚未获得完整命令
        case RECEIVE_CMD:
            pstate = WAIT_HEADER; // 无论后续字节是什么，先复位状态
            // 复用原有解析
            if ((byte & 0xF0) == 0xA0) {
                cmd->type      = CAR_CMD_LOCK;
                cmd->target_id = (byte & 0x0F) + 1;
                return 1;
            }
            switch (byte) {
                case 0x01: cmd->type = CAR_CMD_LEFT;     return 1;
                case 0x02: cmd->type = CAR_CMD_RIGHT;    return 1;
                case 0x00: cmd->type = CAR_CMD_STRAIGHT; return 1;
                case 0xFF: cmd->type = CAR_CMD_NO_TARGET;return 1;
                default: break;
            }
            return 0; // 如果收到非预期的命令字节，丢弃
    }
    return 0;
}


static void ring_buf_init(ring_buf_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    for (int i = 0; i < CAR_UART_RBUF_SIZE; i++)
        rb->buf[i] = 0;
}

static void ring_buf_put_isr(ring_buf_t *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % CAR_UART_RBUF_SIZE;
    if (next != rb->tail) {          // 缓冲区未满
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

    uart_init(CAR_UART_INDEX, CAR_UART_BAUD,
              CAR_UART_TX_PIN, CAR_UART_RX_PIN);

    uart_set_callback(CAR_UART_INDEX, rx_isr, NULL);
    uart_set_interrupt_config(CAR_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

// ===================== 命令解析 =====================
static uint8_t parse_byte(uint8_t byte, car_cmd_t *cmd) {
    // 锁定指令 (0xA0 ~ 0xA7)
    if ((byte & 0xF0) == 0xA0) {
        cmd->type      = CAR_CMD_LOCK;
        cmd->target_id = (byte & 0x0F) + 1;   // 0~7 → 1~8
        return 1;
    }
    // 转向/状态指令
    switch (byte) {
        case 0x01: cmd->type = CAR_CMD_LEFT;     return 1;
        case 0x02: cmd->type = CAR_CMD_RIGHT;    return 1;
        case 0x00: cmd->type = CAR_CMD_STRAIGHT; return 1;
        case 0xFF: cmd->type = CAR_CMD_NO_TARGET;return 1;
        default: break;
    }
    return 0;   // 无效字节
}

// ===================== 轮询 =====================
uint8_t car_uart_poll(car_cmd_t *out) {
    uint8_t byte;
    while (ring_buf_get(&rbuf, &byte)) {
        if (parse_byte_with_header(byte, out)) {
            // ---- 新增：更新内部状态 ----
            if (out->type == CAR_CMD_LOCK) {
                s_locked_id = out->target_id;           // 记录锁定目标
            } else {
                // 转向命令 → 更新方向值
                switch (out->type) {
                    case CAR_CMD_LEFT:      s_turn_dir = 1; break;
                    case CAR_CMD_RIGHT:     s_turn_dir = 2; break;
                    case CAR_CMD_STRAIGHT:  s_turn_dir = 0; break;   // 直行视为 0
                    case CAR_CMD_NO_TARGET: s_turn_dir = 0; break;
                    default: break;
                }
            }
            // --------------------------------
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




uint8_t car_uart_get_turn_dir(void) {
    // 可选：若未锁定，强制返回 0（可根据需求启用）
    // if (s_locked_id == 0) return 0;
    return s_turn_dir;
}

uint8_t car_uart_get_locked_id(void) {
    return s_locked_id;
}

