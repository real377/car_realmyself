#ifndef __HUIDU_H
#define __HUIDU_H

//void task();
 void Cross();
void huidu_init();
void get_state();
void cross_deal(uint8 mode);
extern uint8 list[8];
extern uint8 temp[8]; 
extern float weight[8];
extern float err;
extern int cross_flag,T_mode,T_flag;
void huidu_err();
#endif