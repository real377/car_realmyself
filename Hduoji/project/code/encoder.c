#include "zf_common_headfile.h"
void L_PIN1_exti_handler(uint32 state, void *ptr);
void L_PIN2_exti_handler(uint32 state, void *ptr);
void R_PIN1_exti_handler(uint32 state, void *ptr);
void R_PIN2_exti_handler(uint32 state, void *ptr);
// 中断函数声明
int16_t L_count = 0,R_count = 0,encoder_flag = 0;
void ctimer_clean(void)
{
		L_count = 0;	
		R_count = 0;
	
		//speed_L=0;
		//speed_R=0;
}
void encoder_init()
{
    exti_init(L_PIN1, EXTI_TRIGGER_BOTH,L_PIN1_exti_handler,NULL);                                      
    exti_init(L_PIN2, EXTI_TRIGGER_BOTH,L_PIN2_exti_handler,NULL);                                      
    exti_init(R_PIN1, EXTI_TRIGGER_BOTH,R_PIN1_exti_handler,NULL);                                      
    exti_init(R_PIN2, EXTI_TRIGGER_BOTH,R_PIN2_exti_handler,NULL);                                     	
}	
void L_PIN1_exti_handler(uint32 state, void *ptr)
{
	if(gpio_get_level(L_PIN1)) 
	{  // 确认仍是有效边沿
		uint8 pin2 = gpio_get_level(L_PIN2);
    if (pin2) L_count++;
    else L_count--;
	}
}
void L_PIN2_exti_handler(uint32 state, void *ptr)
{
	if(gpio_get_level(L_PIN2)) 
	{  // 确认仍是有效边沿
		uint8 pin1 = gpio_get_level(L_PIN1);
    if (pin1) L_count--;
    else L_count++;
	}
}
void R_PIN1_exti_handler(uint32 state, void *ptr)
{
	if(gpio_get_level(R_PIN1)) 
	{  // 确认仍是有效边沿
		uint8 pin2 = gpio_get_level(R_PIN2);
    if (pin2) R_count++;
    else R_count--;
	}

}
void R_PIN2_exti_handler(uint32 state, void *ptr)
{
	if(gpio_get_level(R_PIN2)) 
	{  // 确认仍是有效边沿
		uint8 pin1 = gpio_get_level(R_PIN1);
    if (pin1) R_count--;
    else R_count++;
	}
}

