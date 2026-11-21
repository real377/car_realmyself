#include "zf_common_headfile.h"


#define BUFFER_SIZE    64        // 缓冲区大小，根据需求调整

uint8_t rx_buffer[BUFFER_SIZE];  // 接收缓冲区数组
uint8_t rx_index = 0;            // 当前写入位置的索引
extern uint8 re_data;

void my_uart_rx_callback(uint32_t param, void *ptr)
{
    // 直接处理接收到的数据
    // 例如：存到缓冲区、判断帧头、协议解析等
    
    // 如果需要回显测试：
    // uart_write_byte(UART_2, receiveData);
	
	re_data=param;
	// 判断确实是接收中断
    if (param == UART_INTERRUPT_STATE_RX)  // 可能需要确认这个宏名
    {
        // 直接读取数据寄存器
        uint8_t receiveData = DL_UART_Main_receiveData(UART2);
        
        // 处理数据
        // 例如：存到缓冲区
        rx_buffer[rx_index++] = receiveData;
        rx_index %= BUFFER_SIZE;
    }
}

void uart2_init(void)
{
    uart_init(UART_2, 9600, UART2_TX_A21, UART2_RX_B18);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
    interrupt_set_priority(UART2_INT_IRQn, 2);
    
    uart_set_callback(UART_2, my_uart_rx_callback,NULL);
}

/**
 * @brief   发送数据协议: b3 b3 data1 data2 data3 5b
 * @param   str     要发送的3字节数据指针
 * @retval  无
 */
void u2_sendData(uint8_t *str)
{
    uint8_t i = 0;
    
    // 发送 0xb3 两次（逐飞库的阻塞发送已包含等待TC逻辑）
    for (i = 0; i < 2; i++)
    {
        uart_write_byte(UART_2, 0xb3);  // 阻塞发送，内部已等待发送完成
    }
    
    // 发送3位有效数据
    for (i = 0; i < 3; i++)
    {
        uart_write_byte(UART_2, str[i]);
    }
    
    // 发送结束符 0x5b
    uart_write_byte(UART_2, 0x5b);
}

///**
// * @brief   串口2接收中断服务函数
// * @param   无
// * @retval  无
// */
//void UART2_IRQHandler(void)
//{
//    uint8_t receiveData;
//    
//    // 检查接收中断标志（逐飞库的查询函数）
//    if (uart_query_byte(UART_2, &receiveData))
//    {
//        // 接收到数据，在这里处理
//        // 例如：存到缓冲区、判断帧头等
//        
//        // 如果需要回显测试：
//        // uart_write_byte(UART_2, receiveData);
//        
//        // 如果你的硬件有LED指示灯，可以在这里控制
//        // 例如：MY_GPIO_Init(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);
//    }
//}

