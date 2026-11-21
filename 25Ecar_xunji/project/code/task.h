#ifndef __TASK_H
#define __TASK_H

uint8_t read_line_sensors();
uint8_t read_line_sensors2();
void buzzer(uint8_t mode);
void buzzer_2();
void task1();
void task2();
void task3();
void task3_2();
void task4();
void task1_ti(void);
void task12();
void tracking(void);

typedef enum {
    PROC_NONE = 0,
    PROC_STRAIGHT = 1,
    PROC_TURN = 2
} CornerProcessState;


// ===== 圈数计数 =====
extern int  lap_target;
extern int  lap_current;
extern int  corner_cnt;
extern int  lap_stop;
extern int  lap_running;
void lap_counter(void);


// 新增状态机所需变量
extern CornerProcessState cp_state;
extern int32_t avg_dist;


extern int count6;
extern int flag1;
extern int count_4;
extern int16 ceshi;
extern int flag;
extern float ceshi_2;
extern float ceshi_3;
extern int zhijiao_flag;
extern int plus_flag; 
extern int Flag_B;
#endif