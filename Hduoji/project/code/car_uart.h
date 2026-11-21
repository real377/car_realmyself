/*****************************************************************************
 * 文件名称: car_uart.h
 * 功能描述: 接收 K230 发来的偏差+速度数据包 (多字节帧)
 * 协议: 0xAA + 偏差H + 偏差L + 速度 + 异或校验
 * 依赖: 逐飞库 zf_driver_uart
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

// 偏差值缩放因子：收到的原始值 / 100 = cm
#define CAR_DEVIATION_SCALE     100.0f

// ======================= 数据结构 =======================
typedef struct {
    int16_t deviation;          // 偏差原始值 (单位: 0.01cm)
    int8_t  speed;              // 钢球速度 (单位: cm/s, 有符号)
} car_data_t;


// ======================= API =======================
void     car_uart_init          (void);                             // 初始化 UART (中断接收)
uint8_t  car_uart_poll          (car_data_t *out);                  // 尝试解析一帧数据, 返回1=有新数据
uint16_t car_uart_byte_count    (void);                             // 已接收字节总数 (调试用)
void     car_uart_send_byte     (uint8_t byte);                     // 发送单字节
void     car_uart_send_str      (const char *str);                  // 发送字符串

// 直接读取最新值（非阻塞）
uint8_t  car_uart_is_new_data   (void);                             // 是否有新数据
car_data_t car_uart_get_data    (void);                             // 获取最新数据（不清除标志）

// 获取最近一帧的原始 5 字节（调试用），返回 1 表示有新数据
uint8_t car_uart_get_raw_frame(uint8_t buf[5]);

extern car_data_t s_latest_data;
#endif