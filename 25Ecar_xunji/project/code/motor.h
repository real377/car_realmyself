#ifndef __MOTOR_H
#define __MOTOR_H	    
#define PWM_L         PWM_TIM_A0_CH3_A28      //PWM输出引脚
#define PWM_R         PWM_TIM_A1_CH1_B3    //PWM输出引脚
#define DIR_L1   			A31             //电机使能引脚
#define DIR_L2   			B4                 //电机使能引脚
#define DIR_R1  		 	B2		    //电机使能引脚
#define DIR_R2  		 	B16 		    //电机使能引脚


#define PulseMPS		4.5*3.14/680                               
typedef struct
{
    uint8 flag;         
    float Lenth;     
} Step;

typedef struct
{
    
    int16 target_val,   
         now;  //反馈值 

    
    int16 ek,          
          ek1;    
    
    float goal;
    float pid_delta;   
    int16 output_val,
          output_val_last,
              bang;
    
    float kp, 
          ki;   
				

       
    
} PID;
void Get_Speed();
void out_pwm(int32 dutyL, int32 dutyR);//左 右
void motor_turn();
void zhijiao_turn();
void plus_turn();
void PID_motor(PID *pid);
void PID_Motor_Init(void);
void line();
void stop();
void Motor_Speed_Control(void);
void motor_turn_with_speed(int16 target_speed);
void Motor_Speed_Control111(void);




// motor.h 增加内容
extern PID PidLeft;   // 左轮速度 PID
extern PID PidRight;  // 右轮速度 PID
extern PID Speed;
extern Step Step1;
extern float lenght;
extern int speed_L ,speed_R,plus ;
extern int32 dutyL,dutyR;


#endif