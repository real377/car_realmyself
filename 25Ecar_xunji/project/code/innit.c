#include "zf_common_headfile.h"
void Car_Init()
{
	  
	
		gpio_init(DIR_L1, GPO, 1, GPO_PUSH_PULL);
		gpio_init(DIR_L2, GPO, 1, GPO_PUSH_PULL);
		gpio_init(DIR_R1, GPO, 1, GPO_PUSH_PULL);
		gpio_init(DIR_R2, GPO, 1, GPO_PUSH_PULL);
	
		gpio_init(L_PIN1, GPI, 0, GPI_PULL_DOWN);
		gpio_init(L_PIN2, GPI, 0, GPI_PULL_DOWN);
		gpio_init(R_PIN1, GPI, 0, GPI_PULL_DOWN);
		gpio_init(R_PIN2, GPI, 0, GPI_PULL_DOWN);

//		ips200_init(IPS200_TYPE_SPI);	
		OLED_Init();
		huidu_init();	
		//imu660ra_init();
//		imu660rb_init();
//		GyroOffsets(1500);	
		encoder_init();
		PID_Motor_Init();
		DerectPid_Init();
		pwm_init(PWM_L,10000,0);
		pwm_init(PWM_R,10000,0);

}