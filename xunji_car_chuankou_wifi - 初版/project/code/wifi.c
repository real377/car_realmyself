/*****************************************************************************
 * 文件名称: wifi.c
 * 功能描述: ESP-01S AT指令非透传TCP通信模块（非阻塞收发版）
 * 说明:
 *  - 使用UART中断接收，环形缓冲 + 状态机解析
 *  - 发送采用队列 + 非阻塞状态机，与接收并行，不会因等待响应而丢数据
 *  - 发送失败自动重试1次（共2次尝试）
 *  - wifi_poll() 需要在主循环中频繁调用，完成收/发处理
 ****************************************************************************/
#include "wifi.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ==================== 内部定义 ==================== */
#define LINK_ID            0

/* 环形缓冲区（ESP-01S → MCU） */
#define RBUF_SIZE          512

/* 发送超时配置（毫秒） */
#define TIMEOUT_WAIT_PROMPT    2000    /* 等待 '>'                       */
#define TIMEOUT_WAIT_SEND_OK   4000    /* 等待 "SEND OK"                 */
#define SEND_RETRY_MAX         1       /* 总尝试次数 = SEND_RETRY_MAX + 1 */

/* 发送状态机状态 */
typedef enum {
    SEND_IDLE,              /* 无发送任务 */
    SEND_WAIT_PROMPT,       /* 已发 AT+CIPSEND，正在等待 '>'          */
    SEND_WAIT_OK            /* 已发数据，正在等待 "SEND OK"            */
} send_state_t;

/* 发送队列元素 */
typedef struct {
    uint8_t data[WIFI_SEND_BUF_SIZE];
    uint16_t len;
} send_item_t;

/* ==================== 全局变量 ==================== */
static struct {
    uint8_t  data[RBUF_SIZE];
    volatile uint16_t head;
    uint16_t tail;
} rbuf;

/* 应用数据接收缓冲区 */
static uint8_t  rx_data[WIFI_RECV_BUF_SIZE];
static uint8_t  rx_len = 0;
static bool     rx_ready = false;

/* TCP连接状态 */
static bool connected = false;

/* ------- 发送队列相关 ------- */
static send_item_t send_queue[WIFI_SEND_QUEUE_SIZE];
static volatile uint8_t send_q_head = 0;    /* 下一个要取出的位置 */
static volatile uint8_t send_q_tail = 0;    /* 下一个要写入的位置 */
static volatile bool    send_q_full = false;

/* 当前正在处理的发送任务 */
static struct {
    send_item_t item;               /* 数据副本 */
    send_state_t state;             /* 当前状态 */
    uint8_t retry_cnt;              /* 已重试次数 */
    uint32_t wait_start;            /* 状态开始时的毫秒时间 */
    uint8_t match_idx;              /* 匹配响应字符串时的当前索引 */
    uint8_t match_buf[16];          /* 暂存已匹配的字符，用于失败时回吐 */
    uint8_t match_len;             /* match_buf 中有效长度 */
} send_ctx;

/* ==================== 状态机解析 + 变量前置 ==================== */
static enum {
    PARSE_IDLE,
    PARSE_PLUS,
    PARSE_I, PARSE_P, PARSE_D,
    PARSE_COMMA1,
    PARSE_LINK,
    PARSE_COMMA2,
    PARSE_LEN,
    PARSE_COLON,
    PARSE_DATA
} ipd_state = PARSE_IDLE;

static uint16_t expected_len = 0;
static uint8_t  link_id_tmp = 0;

/* ==================== 环形缓冲操作（带临界区保护） ==================== */
static void rbuf_put(uint8_t byte) {
    uint16_t next = (rbuf.head + 1) % RBUF_SIZE;
    __disable_irq();
    if (next != rbuf.tail) {
        rbuf.data[rbuf.head] = byte;
        rbuf.head = next;
    }
    __enable_irq();
}

static uint8_t rbuf_get(uint8_t *byte) {
    uint8_t ret = 0;
    __disable_irq();
    if (rbuf.tail != rbuf.head) {
        *byte = rbuf.data[rbuf.tail];
        rbuf.tail = (rbuf.tail + 1) % RBUF_SIZE;
        ret = 1;
    }
    __enable_irq();
    return ret;
}

static void rbuf_clear(void) {
    __disable_irq();
    rbuf.head = 0;
    rbuf.tail = 0;
    __enable_irq();
}


/* ==================== 等待指定字符串（仅初始化阶段使用） ==================== */
static bool wait_for_string(const char *target, uint32_t timeout_ms) {
    uint8_t  byte;
    uint8_t  match_idx = 0;
    uint8_t  target_len = strlen(target);
    uint32_t elapsed = 0;
    const uint32_t delay_per_loop = 1;   // 1ms

    while (elapsed < timeout_ms) {
        if (rbuf_get(&byte)) {
            if (byte == target[match_idx]) {
                match_idx++;
                if (match_idx == target_len) {
                    return true;
                }
            } else {
                match_idx = 0;
                if (byte == target[match_idx]) match_idx = 1;
            }
        } else {
            system_delay_ms(1);
            elapsed += delay_per_loop;
        }
    }
    return false;
}


/* ==================== 发送辅助函数 ==================== */
static void uart_send_str(const char *str) {
    uart_write_string(WIFI_UART_INDEX, str);
}

static void wifi_at_send(const char *cmd) {
    uart_send_str(cmd);
    uart_send_str("\r\n");
}

/* ==================== 发送队列操作 ==================== */
static bool send_queue_push(const uint8_t *data, uint16_t len) {
    if (len > WIFI_SEND_BUF_SIZE) return false;

    __disable_irq();
    if (send_q_full) {
        __enable_irq();
        return false;   /* 队列满 */
    }

    uint8_t next_tail = (send_q_tail + 1) % WIFI_SEND_QUEUE_SIZE;
    if (next_tail == send_q_head) {   /* 写入后队满 */
        send_q_full = true;
    }

    memcpy(send_queue[send_q_tail].data, data, len);
    send_queue[send_q_tail].len = len;
    send_q_tail = next_tail;
    __enable_irq();
    return true;
}

static bool send_queue_pop(send_item_t *item) {
    __disable_irq();
    if (send_q_head == send_q_tail && !send_q_full) {
        __enable_irq();
        return false;   /* 队列空 */
    }

    *item = send_queue[send_q_head];
    send_q_head = (send_q_head + 1) % WIFI_SEND_QUEUE_SIZE;
    send_q_full = false;
    __enable_irq();
    return true;
}

static bool send_queue_is_empty(void) {
    __disable_irq();
    bool empty = (send_q_head == send_q_tail && !send_q_full);
    __enable_irq();
    return empty;
}

/* ==================== 启动一个发送任务 ==================== */
static void send_start(void) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d", LINK_ID, send_ctx.item.len);
    wifi_at_send(cmd);
    send_ctx.state = SEND_WAIT_PROMPT;
    send_ctx.match_idx = 0;
    send_ctx.match_len = 0;
    send_ctx.wait_start = millis();
}

/* ==================== 将匹配缓冲中的字节回吐给IPD解析 ==================== */
static void feed_bytes_to_ipd(const uint8_t *bytes, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = bytes[i];
        switch (ipd_state) {
            case PARSE_IDLE:
                if (b == '+') ipd_state = PARSE_PLUS;
                break;
            case PARSE_PLUS:
                ipd_state = (b == 'I') ? PARSE_I : PARSE_IDLE;
                break;
            case PARSE_I:
                ipd_state = (b == 'P') ? PARSE_P : PARSE_IDLE;
                break;
            case PARSE_P:
                ipd_state = (b == 'D') ? PARSE_D : PARSE_IDLE;
                break;
            case PARSE_D:
                if (b == ',') {
                    ipd_state = PARSE_COMMA1;
                    link_id_tmp = 0;
                    expected_len = 0;
                } else {
                    ipd_state = PARSE_IDLE;
                }
                break;
            case PARSE_COMMA1:
                if (b >= '0' && b <= '9') {
                    link_id_tmp = link_id_tmp * 10 + (b - '0');
                } else if (b == ',') {
                    ipd_state = PARSE_LEN;
                    expected_len = 0;
                } else {
                    ipd_state = PARSE_IDLE;
                }
                break;
            case PARSE_LEN:
                if (b >= '0' && b <= '9') {
                    expected_len = expected_len * 10 + (b - '0');
                } else if (b == ':') {
                    ipd_state = PARSE_COLON;
                } else {
                    ipd_state = PARSE_IDLE;
                }
                break;
            case PARSE_COLON:
                rx_len = 0;
                ipd_state = PARSE_DATA;
                /* fall through */
            case PARSE_DATA:
                if (rx_len < WIFI_RECV_BUF_SIZE) {
                    rx_data[rx_len++] = b;
                }
                if (rx_len >= expected_len) {
                    rx_ready = true;
                    ipd_state = PARSE_IDLE;
                }
                break;
            default:
                ipd_state = PARSE_IDLE;
                break;
        }
    }
}

/* ==================== 处理发送状态机（一个字节流） ==================== */
static void handle_send_byte(uint8_t byte) {
    if (send_ctx.state == SEND_IDLE) return;

    bool consumed = false;   /* 是否被发送状态机消费 */

    if (send_ctx.state == SEND_WAIT_PROMPT) {
        /* 期望字符: '>' */
        if (byte == '>') {
            /* 成功！将匹配缓冲（空）回吐，然后发送数据 */
            feed_bytes_to_ipd(send_ctx.match_buf, send_ctx.match_len);
            send_ctx.match_len = 0;
            send_ctx.match_idx = 0;

            uart_write_buffer(WIFI_UART_INDEX, send_ctx.item.data, send_ctx.item.len);
            send_ctx.state = SEND_WAIT_OK;
            send_ctx.wait_start = millis();
            consumed = true;
        } else {
            /* 不是 '>'，暂存以便失败时回吐 */
            if (send_ctx.match_len < sizeof(send_ctx.match_buf)) {
                send_ctx.match_buf[send_ctx.match_len++] = byte;
            }
            consumed = true;   /* 仍然消费了，但不匹配 */
        }
    } else if (send_ctx.state == SEND_WAIT_OK) {
        /* 期望字符串: "SEND OK" */
        const char *target = "SEND OK";
        uint8_t target_len = 7;
        if (byte == target[send_ctx.match_idx]) {
            send_ctx.match_buf[send_ctx.match_len++] = byte;
            send_ctx.match_idx++;
            consumed = true;
            if (send_ctx.match_idx == target_len) {
                /* 匹配完成！回吐匹配缓冲之前的内容（不含本字符串）？不对，
                   匹配缓冲中就是完整的"SEND OK"，不应回吐给IPD。
                   但之前的字节（如果有）已经回吐了？这里没有之前字节。
                   匹配成功后直接清空缓冲，发送完成。 */
                send_ctx.match_len = 0;
                send_ctx.match_idx = 0;
                send_ctx.state = SEND_IDLE;
                /* 发送任务完成，稍后由poll启动下一个 */
            }
        } else {
            /* 匹配失败：将 match_buf 中所有已匹配的字节回吐给 IPD */
            feed_bytes_to_ipd(send_ctx.match_buf, send_ctx.match_len);
            send_ctx.match_len = 0;
            send_ctx.match_idx = 0;

            /* 当前字节也不属于发送，交给IPD */
            feed_bytes_to_ipd(&byte, 1);
            /* 注意：此时发送状态机可能仍为 WAIT_OK，但匹配索引已重置，
               超时逻辑会处理失败 */
            consumed = true;   /* 字节已被回吐，不算丢弃 */
        }
    }

    (void)consumed;
}

/* ==================== 检查发送超时 ==================== */
static void check_send_timeout(void) {
    if (send_ctx.state == SEND_IDLE) return;

    uint32_t timeout = (send_ctx.state == SEND_WAIT_PROMPT) ? TIMEOUT_WAIT_PROMPT : TIMEOUT_WAIT_SEND_OK;
    if ((millis() - send_ctx.wait_start) >= timeout) {
        /* 超时 -> 回吐 match_buf 中的字节，然后重试或放弃 */
        feed_bytes_to_ipd(send_ctx.match_buf, send_ctx.match_len);
        send_ctx.match_len = 0;
        send_ctx.match_idx = 0;

        if (send_ctx.retry_cnt < SEND_RETRY_MAX) {
            send_ctx.retry_cnt++;
            send_start();   /* 重新开始当前发送任务 */
        } else {
            /* 重试次数用尽，放弃当前包 */
            send_ctx.state = SEND_IDLE;
        }
    }
}

/* ==================== UART 中断回调 ==================== */
static void wifi_uart_isr(uint32_t event, void *ptr) {
    (void)event;
    (void)ptr;
    uint8_t byte;
    while (uart_query_byte(WIFI_UART_INDEX, &byte) == ZF_TRUE) {
        rbuf_put(byte);
    }
}

/* ==================== 初始化流程 ==================== */
bool wifi_init(uint8_t role) {
    uart_init(WIFI_UART_INDEX, WIFI_UART_BAUD, WIFI_TX_PIN, WIFI_RX_PIN);
    uart_set_callback(WIFI_UART_INDEX, wifi_uart_isr, NULL);
    uart_set_interrupt_config(WIFI_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    rbuf_clear();
    rx_ready = false;
    connected = false;
    send_ctx.state = SEND_IDLE;
    send_q_head = send_q_tail = 0;
    send_q_full = false;

    // 测试模块
    wifi_at_send("AT");
    if (!wait_for_string("OK", 2000)) return false;   /* wait_for_string 仅初始化时用，不再用于运行时 */

    if (role == WIFI_ROLE_AP_SERVER) {
        wifi_at_send("AT+CWMODE_DEF=2");
        if (!wait_for_string("OK", 3000)) return false;
        wifi_at_send("AT+CWSAP_DEF=\"ESP01_CAR\",\"12345678\",5,3");
        if (!wait_for_string("OK", 5000)) return false;
        wifi_at_send("AT+CIPMUX=1");
        if (!wait_for_string("OK", 2000)) return false;
        wifi_at_send("AT+CIPSERVER=1,8080");
        if (!wait_for_string("OK", 2000)) return false;
        if (!wait_for_string("0,CONNECT", 15000)) return false;
        connected = true;
    } else {
        wifi_at_send("AT+CWMODE_DEF=1");
        if (!wait_for_string("OK", 3000)) return false;
        wifi_at_send("AT+CWJAP_DEF=\"ESP01_CAR\",\"12345678\"");
        if (!wait_for_string("WIFI GOT IP", 15000)) return false;
        wifi_at_send("AT+CIPMUX=0");
        if (!wait_for_string("OK", 2000)) return false;
        wifi_at_send("AT+CIPSTART=\"TCP\",\"192.168.4.1\",8080");
        if (!wait_for_string("CONNECT", 10000)) return false;
        connected = true;
    }

    // 清空残留
    rbuf_clear();
    ipd_state = PARSE_IDLE;
    expected_len = 0;
    link_id_tmp = 0;
    rx_ready = false;
    rx_len = 0;

    return true;
}

/* ==================== 发送数据（入队接口） ==================== */
bool wifi_send(const uint8_t *data, uint16_t len) {
    if (!connected) return false;
    return send_queue_push(data, len);
}

/* ==================== 发送队列是否全部完成 ==================== */
bool wifi_send_all_done(void) {
    if (send_ctx.state != SEND_IDLE) return false;
    return send_queue_is_empty();
}

/* ==================== 接收处理（while循环中调用） ==================== */
void wifi_poll(void) {
    uint8_t byte;

    /* 1. 从环形缓冲读取一个字节 */
    if (rbuf_get(&byte)) {
        /* 如果发送忙，优先尝试用该字节满足发送状态机；
           若发送状态机未消费（或被回吐），再传给IPD解析 */
        bool send_consumed = false;
        if (send_ctx.state != SEND_IDLE) {
            /* handle_send_byte 内部会决定消费与否，并负责回吐 */
            uint8_t pre_match_len = send_ctx.match_len;   /* 记录匹配缓冲长度变化 */
            handle_send_byte(byte);
            /* 如果匹配缓冲长度增加，说明字节被存入匹配缓冲，认为已消费；
               注意：匹配失败时也会被回吐，但此时长度会归零，不算消费 */
            if (send_ctx.match_len > pre_match_len || send_ctx.state == SEND_WAIT_OK) { // 简单判断
                send_consumed = true;
            } else {
                /* 字节已被回吐，需交由IPD再次处理 */
            }
        }
        if (!send_consumed) {
            /* 传递给IPD解析 */
            feed_bytes_to_ipd(&byte, 1);
        }
    }

    /* 2. 检查发送超时（可能触发重试） */
    check_send_timeout();

    /* 3. 如果发送空闲且队列非空，启动下一个发送任务 */
    if (send_ctx.state == SEND_IDLE && !send_queue_is_empty()) {
        send_queue_pop(&send_ctx.item);
        send_ctx.retry_cnt = 0;
        send_start();
    }
}

/* ==================== 接收数据读出 ==================== */
uint8_t wifi_recv(uint8_t *buf, uint8_t max_len) {
    if (!rx_ready) return 0;
    uint8_t len = rx_len < max_len ? rx_len : max_len;
    memcpy(buf, rx_data, len);
    rx_ready = false;
    rx_len = 0;
    return len;
}

/* ==================== 连接状态 ==================== */
bool wifi_is_connected(void) {
    return connected;
}