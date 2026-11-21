#ifndef _DIERCTION_H
#define _DIERCTION_H

typedef struct
{
    float kp,//比例
          kd,//微分
          out_kp,//差值权值一般不用改
          out_ki,//陀螺仪权值改变阻尼大小代i
          in_kp,//adc本次权值
          in_kd;//adc上次权值
    
    float target_val,   //目标值
          target_val_last;//上次目标值
    
    float in_ek,
					ek,          //偏差值
          ek1,     //上一个偏差值
          ek2;     //上上个偏差值
    
    float output_val,//输出值
          output_val_last,
          maxOutput;//输出、输出限幅
    
    uint8 COUNTS;//内外环计数器
}turn;


void Direction_Reset(void);

extern turn price;
void DerectPid_Init();
void DerectError_Get();
//void DerectError_Get2();
//void DerectError_Get3();
void Direction_out();
void  DifferControl();

#endif