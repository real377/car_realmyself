#ifndef __HUIDU_H
#define __HUIDU_H

//void task();
 void Cross();
void huidu_init();
void get_state();
void cross_deal(uint8 mode);
extern uint8 list[7];
extern uint8 temp[7]; 
extern float weight[7];
extern float err;
extern int cross_flag,T_mode,T_flag;
extern uint8_t imu_correction_enable;   // 1：IMU参与方向修正；0：不参与

extern int corner_state;   // ★新增


void huidu_err();
#endif