/*****************************************************************************
 * 文件名称: wifi.h
 * 功能描述: ESP-01S AT指令非透传TCP通信模块（非阻塞收发版）
 * 硬件依赖: 逐飞库 UART（MSPM0G3507）
 ****************************************************************************/
#ifndef __WIFI_H__
#define __WIFI_H__

#include "zf_common_headfile.h"
#include <stdbool.h>

/* ========== 用户配置 ========== */
//#define WIFI_UART_INDEX      UART_1          /* 需在外部定义或取消注释 */
//#define WIFI_UART_BAUD       115200
#define WIFI_TX_PIN          UART1_TX_A8
#define WIFI_RX_PIN          UART1_RX_A9

#define WIFI_RECV_BUF_SIZE   128             /* 接收缓冲区大小 */
#define WIFI_SEND_BUF_SIZE   256             /* 发送数据缓冲区大小（单个包） */
#define WIFI_SEND_QUEUE_SIZE 4               /* 发送队列深度 */

/* ========== 角色定义 ========== */
#define WIFI_ROLE_AP_SERVER  0
#define WIFI_ROLE_STA_CLIENT 1

/* ========== 通信命令 ========== */
#define WIFI_CMD_CONTINUE    0xC1
#define WIFI_CMD_GO_FETCH    0xC2

/* ========== 函数接口 ========== */
bool     wifi_init(uint8_t role);
bool     wifi_send(const uint8_t *data, uint16_t len);   /* 将数据加入发送队列，立即返回 */
uint8_t  wifi_recv(uint8_t *buf, uint8_t max_len);
void     wifi_poll(void);                               /* 在主循环中频繁调用 */
bool     wifi_is_connected(void);
bool     wifi_send_all_done(void);                      /* 发送队列是否为空且无正在发送 */

/* ========== 需用户实现的毫秒计数器 ========== */
extern uint32_t millis(void);   /* 返回系统上电后的毫秒数，需在main.c中实现（如基于SysTick） */

#endif /* __WIFI_H__ */