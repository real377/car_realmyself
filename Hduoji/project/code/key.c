#include "zf_common_headfile.h"


void my_Key_Init(void)
{
    //gpio_init (A1, GPI, GPIO_HIGH, GPI_PULL_UP);
//	gpio_init (B14, GPI, GPIO_HIGH, GPI_PULL_UP);
	
	gpio_init (A27, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init (A25, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init (B21, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init (A30, GPI, GPIO_HIGH, GPI_PULL_UP);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (gpio_get_level(A27) == 0)
	{
//		system_delay_ms(20);
//		while (gpio_get_level(B14) == 0)
//			;
		system_delay_ms(20);
		KeyNum = 1;
	}
	if (gpio_get_level(A25) == 0)
	{
//		system_delay_ms(20);
//		while (gpio_get_level(B14) == 0)
//			;
//		system_delay_ms(20);
		KeyNum = 2;
	}
	if (gpio_get_level(B21) == 0)
	{
//		system_delay_ms(20);
//		while (gpio_get_level(B14) == 0)
//			;
//		system_delay_ms(20);
		KeyNum = 3;
	}
	if (gpio_get_level(A30) == 0)
	{
//		system_delay_ms(20);
//		while (gpio_get_level(B14) == 0)
//			;
//		system_delay_ms(20);
		KeyNum = 4;
	}
	

	/*
	else if (gpio_get_level(A1) == 0)
	{
		system_delay_ms(20);
		while (gpio_get_level(A1) == 0)
			;
		system_delay_ms(20);
		KeyNum = 2;
	}
	else if (gpio_get_level(A1) == 0)
	{
		system_delay_ms(20);
		while (gpio_get_level(A1) == 0)
			;
		system_delay_ms(20);
		KeyNum = 3;
	}
   */
	return KeyNum;
}
