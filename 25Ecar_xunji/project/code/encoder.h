#ifndef _ENCODER_H
#define _ENCODER_H

#define L_PIN1 B5
#define L_PIN2 A10
#define R_PIN1 A9                                       
#define R_PIN2 B15 

void encoder_init(void);
void ctimer_clean(void);

extern int16_t L_count, R_count, encoder_flag;
extern int32_t L_pulse_sum;   // 左轮总脉冲（不清零）
extern int32_t R_pulse_sum;   // 右轮总脉冲（不清零）

#endif