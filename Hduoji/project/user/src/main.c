/*********************************************************************************************************************
* MSPM0G3507 Opensource Library（MSPM0G3507 开源库）：基于官方 SDK 接口的底层开源库
* Copyright (c) 2022 SEEKFREE 成都逐飞科技
*
* 本文件是 MSPM0G3507 开源库的一部分
*
* MSPM0G3507 开源库 可自由使用
* 如有疑问请联系逐飞科技或在官方论坛发帖讨论
* 但不允许修改后随意公开发布
*
* 本开源库的发布希望能够帮助大家，但未提供任何的保证
* 对于用户使用过程中出现的任何问题不承担任何责任
* 更多详情请参见 GPL
*
* 您应该已经收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请访问 <https://www.gnu.org/licenses/>
*
* 版权说明：
* 本开源库使用 GPL3.0 开源许可协议 请遵守协议中规定的各类条款
* 具体英文版参见 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 许可证明在 libraries 文件夹 当前文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本开源库 但修改时请保留逐飞科技的版权声明等信息
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹下 version 文件 版本说明
* 编译环境          MDK 5.37
* 编译平台          MSPM0G3507
* 技术论坛          https://seekfree.taobao.com/
*********************************************************************************************************************/

#include "zf_common_headfile.h"

#include "duoji.h"
#include "car_uart.h"
#include "OLED.h"

/* ====== 全局变量 ====== */
uint8 data[3] = {0, 0, 0};
uint8 flag_1;

float servo_out = 0;
float servo_Kp = 0 , servo_Kd = 0;
int16 target = 0 , err_now = 0 , err_last = 0;

void pit_handler_1 (uint32 state, void *ptr);
void pit_handler_2 (uint32 state, void *ptr);

car_data_t k230_data;
/* ====== 舵机控制 —— 由 duoji.c 统一管理 ====== */
int16 pwm_ceshi   = 0;   /* OLED: 当前 PWM 偏移 */
int16 pwm_ceshi_2 = 0;   /* OLED: 摄像头方向 / 模式状态 */
int16 dianping = 0;
int16 jiajia = 0;
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

void PID_servo(float ki ,float kd , int16 target)
{
	
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);
    Car_Init();
    duoji_set_mode(DUOJI_MODE_DYNAMIC);  /* 默认摄像头触发模式，按键可切换 AUTO */

	interrupt_set_priority (TIMA0_INT_IRQn, 1);
	interrupt_set_priority (TIMA1_INT_IRQn, 2);
    pit_ms_init(PIT_TIM_A0, 5, pit_handler_1, NULL);
    pit_ms_init(PIT_TIM_A1, 5, pit_handler_2, NULL);
	
	char str[17];
    while (true)
    {
//        sprintf(buffer, "dis:%d",  k230_data.deviation);
//        OLED_ShowString(1, 1, buffer);
//        sprintf(buffer, "spe:%d",  k230_data.speed);
//        OLED_ShowString(2, 1, buffer);
//        sprintf(buffer, "dir:%d",  pwm_ceshi_2);
//        OLED_ShowString(3, 1, buffer);
//        sprintf(buffer, "key:%d",  key);
//        OLED_ShowString(4, 1, buffer);
		
		 // ---- 行2：偏差值与速度 ----
            int_to_str(k230_data.deviation, str, 6);
            OLED_ShowString(2, 1, "D:");
            OLED_ShowString(2, 3, str);                     // 偏差原始值
            int_to_str(k230_data.speed, str, 3);
            OLED_ShowString(3, 1, "S:");
            OLED_ShowString(3, 3, str);
	}
}

/* PIT 定时中断回调（每 3ms 触发） */
void pit_handler_1 (uint32 state, void *ptr)
{

   
	jiajia++;
	jiajia = func_limit_ab(jiajia,0,800);
	pwm_set_duty(SERVO_2, SERVO2_MID - jiajia);
}

void pit_handler_2 (uint32 state, void *ptr)
{
//    if (car_uart_poll(&k230_data)) {
//        int16 dev = k230_data.deviation;
//        int16 amp = (dev >= 0) ? dev : -dev;

//        /* 死区：偏差太小不触发 */
////	if (dev > 0)
////	{
////        if(amp < 50) return;
////		if(amp >= 50 && amp < 100)
////		{
////			amp=1;
////			duoji_set_sweep_amp(200);
////		}
////        if(amp >= 100 && amp < 300)
////		{
////			amp=1;
////			duoji_set_sweep_amp(200);
////		}	
////		if(amp >= 300 && amp < 700)
////		{
////			amp=2;
////			duoji_set_sweep_amp(250);
////		}
////     	if(amp >= 700 && amp < 1200)
////		{
////			amp=3;
////			duoji_set_sweep_amp(300);
////		}	
////		duoji_set_sweep_step(amp*2, amp *1);
////	}
//	
//	if (dev < 0)
//	{
//        if(amp < 50) return;
//		if(amp >= 50 && amp < 100)
//		{
//			amp=1;
//			duoji_set_sweep_amp(200);
//		}
//        if(amp >= 100 && amp < 300)
//		{
//			amp=1;
//			duoji_set_sweep_amp(200);
//		}	
//		if(amp >= 300 && amp < 700)
//		{
//			amp=2;
//			duoji_set_sweep_amp(250);
//		}
//     	if(amp >= 700 && amp < 1200)
//		{
//			amp=3;
//			duoji_set_sweep_amp(300);
//		}	
//		duoji_set_sweep_step(amp*2, amp *1);
//	}
//        

//        /* 速度 → 步长（回程 = 去程 × 2，限幅 2~30） */
////	int16 spd = (int16)k230_data.speed;
////	if (spd < 0)  spd = -spd;
////	if (spd < 2)  spd = 2;
////	if (spd > 30) spd = 30;
//        

//        /* 偏差方向 → 触发方向 */
////        if (dev > 0) {
////            duoji_set_camera_cmd(DUOJI_CAMERA_UP);   //降得慢，升得快
////        } 
//		 if (dev < 0) {
//            duoji_set_camera_cmd(DUOJI_CAMERA_DOWN);
//        }
//    }
}



//    /* ---- 1. 舵机控制更新 ---- */
//    duoji_update(5);
	
//    pwm_ceshi = duoji_get_pwm_offset();
//   if(gpio_get_level(B22)==1)
//	  dianping = 1;


//    /* ---- 3. 按键循环切换模式 ----
//     * AUTO → CAMERA → PID → SWEEP → DYNAMIC → AUTO → ...
//     */