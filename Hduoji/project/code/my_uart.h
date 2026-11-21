#ifndef __MY_URAT_H
#define __MY_URAT_H

void my_uart_rx_callback(uint32_t param, void *ptr);
void uart2_init(void);
void u2_sendData(uint8_t *str);

#endif
