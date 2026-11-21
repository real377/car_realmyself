#ifndef _ENCODER_H
#define _ENCODER_H



//#define L_PIN1 A0   //´ó³µ
//#define L_PIN2 A1
//#define R_PIN1 B15                                        
//#define R_PIN2 A8
#define L_PIN1 A8
#define L_PIN2 B15
#define R_PIN1 A1                                     
#define R_PIN2 A0

void encoder_init(void);
void ctimer_clean(void);

extern int16_t L_count,R_count,encoder_flag,count1,count2;

#endif