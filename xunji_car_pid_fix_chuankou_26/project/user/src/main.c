/*****************************************************************************
 * 文件名称: main.c
 * 功能描述: UART 调试主函数，显示偏差、速度、原始字节、字节计数
 ****************************************************************************/
#include "zf_common_headfile.h"
#include "car_uart.h"
#include "OLED.h"


int flag;
car_data_t data;

void p_handler(){

		car_uart_poll(&data);
	  flag = 1;


}






/* 将整数转换为字符串（支持负数，右对齐宽度） */
static void int_to_str(int num, char *buf, int width) {
    int i = 0, j, negative = 0;
    char temp[7];
    if (num < 0) { negative = 1; num = -num; }
    do { temp[i++] = (num % 10) + '0'; num /= 10; } while (num && i < 6);
    if (negative) temp[i++] = '-';
    while (i < width) temp[i++] = ' ';
    for (j=0; j<i; j++) buf[j] = temp[i-j-1];
    buf[i] = '\0';
}

/* 字节转十六进制字符串（2字符） */
static void byte_to_hex_str(uint8_t byte, char *str) {
    const char hex[] = "0123456789ABCDEF";
    str[0] = hex[byte >> 4];
    str[1] = hex[byte & 0x0F];
    str[2] = '\0';
}

int main(void) {
    clock_init(SYSTEM_CLOCK_80M);
    system_delay_ms(100);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "K230 Diag");

    car_uart_init();
		
    pit_ms_init(PIT_TIM_A0, 1, p_handler, NULL);    /* 1ms: systick + key_scan    */
	
	
    char str[17];

    while (1) {
        // 轮询接收新帧
        if (flag) {
            // ---- 行2：偏差值与速度 ----
            int_to_str(data.deviation, str, 6);
            OLED_ShowString(2, 1, "D:");
            OLED_ShowString(2, 3, str);                     // 偏差原始值
            int_to_str(data.speed, str, 3);
            OLED_ShowString(3, 1, "S:");
            OLED_ShowString(3, 3, str);
            }

            // ---- 行4：字节总数 ----
            int_to_str(car_uart_byte_count(), str, 6);
            OLED_ShowString(4, 1, "Byt:");
            OLED_ShowString(4, 5, str);
        

					}
}