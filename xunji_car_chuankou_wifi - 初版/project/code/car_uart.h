/*****************************************************************************
 * 文件名称: car_uart.h
 * 功能描述: 接收 K230 发来的单字节指令 (锁定/转向)
 * 协议说明:
 *   0xA0 | (target_idx)  锁定目标数字 (低4位索引0~7, 对应数字1~8)
 *   0x01                 左转
 *   0x02                 右转
 *   0x00                 直行
 *   0xFF                 无目标 / 未锁定
 * 依赖: 逐飞库 zf_common_*, zf_driver_uart
 ****************************************************************************/
#ifndef __CAR_UART_H__
#define __CAR_UART_H__

#include "zf_common_headfile.h"

// ======================= 硬件配置 =======================
#define CAR_UART_INDEX          UART_2          // 使用 UART2
#define CAR_UART_BAUD           115200
#define CAR_UART_TX_PIN         UART2_TX_A21    // PA21
#define CAR_UART_RX_PIN         UART2_RX_A22    // PA22

// 环形缓冲区大小
#define CAR_UART_RBUF_SIZE      256

// ======================= 命令定义 =======================
typedef enum {
    CAR_CMD_NONE        = 0x00,
    CAR_CMD_LOCK        = 0xA0, // 高四位 1010
    CAR_CMD_LEFT        = 0x01,
    CAR_CMD_RIGHT       = 0x02,
    CAR_CMD_STRAIGHT    = 0x00,
    CAR_CMD_NO_TARGET   = 0xFF
} car_cmd_type_t;

// ======================= 解析结果结构体 =======================
typedef struct {
    car_cmd_type_t  type;       // 命令类型
    uint8_t         target_id;  // 当 type == CAR_CMD_LOCK 时有效 (1~8)
} car_cmd_t;

// ======================= API =======================
void     car_uart_init      (void);                                   // 初始化 UART (中断接收)
uint8_t  car_uart_poll      (car_cmd_t *out);                         // 轮询解析一个命令, 返回1=有新命令
uint16_t car_uart_byte_count(void);                                   // 已接收字节总数 (调试用)
void     car_uart_send_byte (uint8_t byte);                           // 发送单字节 (调试/回传)
void     car_uart_send_str  (const char *str);                        // 发送字符串
/* 新增：状态查询接口 */
uint8_t car_uart_get_turn_dir(void);      // 0=无效/直行, 1=左, 2=右
uint8_t car_uart_get_locked_id(void);     // 0=未锁定, 1~8=目标号



#endif