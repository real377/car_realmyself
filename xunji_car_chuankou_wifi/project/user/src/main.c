/*****************************************************************************
 * 文件名称: main.c
 * 功能描述: Wi-Fi 通信测试（非阻塞收发版），基于逐飞 PIT 实现 millis()
 *          调试串口 UART3 (PA14 TX, PA13 RX)
 *          需配合电脑端 串口助手(UART3) + 网络助手(TCP Client)
 ****************************************************************************/
#include "zf_common_headfile.h"
#include "wifi.h"
#include "OLED.h"
#include "zf_driver_pit.h"

/* ==================== 调试串口配置 (UART3) ==================== */
#define DBG_UART_INDEX      UART_3
#define DBG_UART_BAUD       115200
#define DBG_UART_TX_PIN     UART3_TX_A14
#define DBG_UART_RX_PIN     UART3_RX_A13
#define DBG_RBUF_SIZE       256

/* 调试串口环形缓冲区 */
static uint8_t  dbg_rbuf[DBG_RBUF_SIZE];
static volatile uint16_t dbg_head = 0;
static uint16_t dbg_tail = 0;

/* ==================== 系统毫秒计数器（由 PIT 中断驱动） ==================== */
static volatile uint32_t sys_millis = 0;

/* ==================== 调试串口中断回调 ==================== */
static void dbg_uart_isr(uint32_t event, void *ptr)
{
    (void)event;
    (void)ptr;
    uint8_t byte;
    while (uart_query_byte(DBG_UART_INDEX, &byte) == ZF_TRUE)
    {
        uint16_t next = (dbg_head + 1) % DBG_RBUF_SIZE;
        if (next != dbg_tail)
        {
            dbg_rbuf[dbg_head] = byte;
            dbg_head = next;
        }
    }
}

/* ==================== 调试串口读写函数 ==================== */
static uint8_t dbg_get_byte(uint8_t *byte)
{
    if (dbg_tail == dbg_head) return 0;
    *byte = dbg_rbuf[dbg_tail];
    dbg_tail = (dbg_tail + 1) % DBG_RBUF_SIZE;
    return 1;
}

static void dbg_send_byte(uint8_t byte)
{
    uart_write_byte(DBG_UART_INDEX, byte);
}

static void dbg_send_str(const char *str)
{
    while (*str) {
        dbg_send_byte(*str++);
    }
}

/* ==================== PIT 中断回调（1ms） ==================== */
static void pit_millis_isr(uint32_t event, void *ptr)
{
    (void)event;
    (void)ptr;
    sys_millis++;
}

/* ==================== 提供给 wifi.c 的 millis() 实现 ==================== */
uint32_t millis(void)
{
    return sys_millis;
}

/* ==================== Wi-Fi 测试主循环 ==================== */
void wifi_test(void)
{
    uint8_t ch;
    uint8_t wifi_buf[WIFI_RECV_BUF_SIZE];
    uint8_t len;

    /* 初始化调试串口 */
    uart_init(DBG_UART_INDEX, DBG_UART_BAUD, DBG_UART_TX_PIN, DBG_UART_RX_PIN);
    uart_set_callback(DBG_UART_INDEX, dbg_uart_isr, NULL);
    uart_set_interrupt_config(DBG_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    dbg_send_str("Debug UART (UART3) Ready\r\n");

    /* 初始化 Wi-Fi (AP Server) */
    dbg_send_str("Initializing Wi-Fi (AP Server)...\r\n");
    OLED_Clear();
    OLED_ShowString(1, 1, "WiFi Init...");

    if (wifi_init(WIFI_ROLE_AP_SERVER))
    {
        dbg_send_str("Wi-Fi init OK!\r\n");
        dbg_send_str("Waiting for data...\r\n");
        OLED_ShowString(2, 1, "Server OK    ");
    }
    else
    {
        dbg_send_str("Wi-Fi init FAILED!\r\n");
        OLED_ShowString(2, 1, "FAILED       ");
        while (1);  // 失败停在此处
    }

    /* 主循环：双向转发 */
    while (1)
    {
        /* 处理 Wi-Fi 收发（必须频繁调用） */
        wifi_poll();

        /* 从调试串口读取数据 → 通过 Wi-Fi 发送 */
        if (dbg_get_byte(&ch))
        {
            if (!wifi_send(&ch, 1))
            {
                OLED_ShowString(3, 1, "TX ERR       ");
            }
        }

        /* 从 Wi-Fi 读取数据 → 发送到调试串口 */
        len = wifi_recv(wifi_buf, WIFI_RECV_BUF_SIZE);
        if (len > 0)
        {
            for (uint8_t i = 0; i < len; i++)
            {
                dbg_send_byte(wifi_buf[i]);
            }
        }
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);  // 时钟配置及系统初始化
    debug_init();                  // 调试串口信息初始化（默认 UART0）
    
    /* OLED 初始化 */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "WiFi Test");

    /* 初始化 1ms PIT 定时器，用于 Wi-Fi 超时计时 */
    pit_ms_init(PIT_TIM_G6, 1, pit_millis_isr, NULL);  // 使用 TIMG6 作为 1ms 定时器

    /* 进入 Wi-Fi 测试 */
    wifi_test();

    while (1);
}