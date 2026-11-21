/*****************************************************************************
 * 文件名称: main.c
 * 功能描述: K230 → MSPM0 串口接收测试，OLED 实时显示（两行）
 * 硬件连接: OLED SCL=B13, SDA=B12 (I2C)
 *           UART2 RX=PA22, TX=PA21
 ****************************************************************************/
#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "car_uart.h"
#include "OLED.h"                           // 你的 OLED 驱动

static car_cmd_t g_latest_cmd;              // 最新命令
static uint8_t   g_locked_id = 0;           // 锁定目标 ID
static char      str_buf[17];               // 字符串缓冲（16字符+结尾）

/* 简单的字符串复制函数（避免 snprintf） */
static void my_strcpy(char *dst, const char *src) {
    while (*src) *dst++ = *src++;
    *dst = '\0';
}

/* OLED 显示更新函数（无条件刷新） */
static void display_update_oled(void)
{
    car_cmd_t cmd;
    uint8_t new_cmd_received = 0;
    /* 读取所有积压命令 */
    while (car_uart_poll(&cmd)) {
        new_cmd_received = 1;
        /* 遇到锁定命令，立即更新锁定ID，不管后面还有什么命令 */
        if (cmd.type == CAR_CMD_LOCK) {
            g_locked_id = cmd.target_id;
        }
        /* 记录最新一条非锁定命令用于转向显示 */
        if (cmd.type != CAR_CMD_LOCK) {
            g_latest_cmd = cmd;   // 转向/状态命令
        }
    }

		
		
    /* 2. 清屏 */
//    OLED_Clear();

    /* ---- 第一行：当前命令 ---- */
    switch (g_latest_cmd.type) {
        case CAR_CMD_LEFT:      my_strcpy(str_buf, "CMD: LEFT   "); break;
        case CAR_CMD_RIGHT:     my_strcpy(str_buf, "CMD: RIGHT  "); break;
        case CAR_CMD_STRAIGHT:  my_strcpy(str_buf, "CMD: STRAIGHT"); break;
        case CAR_CMD_NO_TARGET: my_strcpy(str_buf, "CMD: NO TARGET"); break;
        case CAR_CMD_LOCK:      my_strcpy(str_buf, "CMD: LOCK   "); break;
        default:                my_strcpy(str_buf, "CMD: UNKNOWN "); break;
    }
    OLED_ShowString(1, 1, str_buf);         // 行1，列1

    /* ---- 第二行：锁定 ID ---- */
    my_strcpy(str_buf, "Locked:");
    if (g_locked_id != 0) {
        // 手动拼接数字：Locked:1
        str_buf[7] = g_locked_id + '0';
        str_buf[8] = ' ';
        str_buf[9] = '\0';
    } else {
        str_buf[7] = '-';
        str_buf[8] = '-';
        str_buf[9] = '\0';
    }
    OLED_ShowString(2, 1, str_buf);

    /* 可选的第三行、第四行（如果需要调试，取消注释即可） */
    // OLED_ShowString(3, 1, "Bytes:");
    // OLED_ShowNum(3, 8, car_uart_byte_count(), 5);
    // OLED_ShowString(4, 1, "Raw:0x");
    // OLED_ShowHexNum(4, 8, g_latest_cmd.type, 2);
}

int main(void)
{
    /* 系统时钟 80MHz */
    clock_init(SYSTEM_CLOCK_80M);
    // debug_init();   // 如果不需要串口0调试，可注释
    system_delay_ms(100);

    /* OLED 初始化 */
    OLED_Init();                // 包含上电延时，无需额外等待
    OLED_Clear();
    OLED_ShowString(1, 1, "UART Test");

    /* 串口接收模块初始化 */
    car_uart_init();

    /* 默认命令 */
    g_latest_cmd.type      = CAR_CMD_NO_TARGET;
    g_latest_cmd.target_id = 0;

    while (1)
    {
        display_update_oled();
        system_delay_ms(1);    // 20Hz 刷新，OLED 响应足够
    }
}