#ifndef __MOTOR_H
#define __MOTOR_H	    
//#define PWM_L         PWM_TIM_G7_CH1_A31      //PWM输出引脚   //大车
//#define PWM_R         PWM_TIM_G7_CH0_A28    //PWM输出引脚
//#define DIR_L1   			B12             //电机使能引脚
//#define DIR_L2   			B13                 //电机使能引脚
//#define DIR_R1  		 	B4		    //电机使能引脚
//#define DIR_R2  		 	B5 		    //电机使能引脚

#define PWM_L          PWM_TIM_G7_CH0_A28     //PWM输出引脚
#define PWM_R          PWM_TIM_G7_CH1_A31 //PWM输出引脚
#define DIR_L1   			 B4            //电机使能引脚
#define DIR_L2   			 B5               //电机使能引脚
#define DIR_R1  		     B13	    //电机使能引脚
#define DIR_R2  		 	 B12	    //电机使能引脚


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
void motor_turn_2();
void zhijiao_turn();
void zhijiao_turn_zuozhuan();
void zhijiao_turn_2();
void zhijiao_turn_3();
void zhijiao_turn_4();
void licheng(void);
void licheng_2(void);
void licheng_3(void);
void licheng_A(void);
void licheng_A_2(void) ;
void licheng_back(void) ;
void licheng_B(void);
void licheng_B_2(void);
void licheng_C(void);
void licheng_C_2(void); 
void licheng_C_back(void);
void plus_turn();
void PID_motor(PID *pid);
void PID_Motor_Init(void);
extern PID Speed;
extern Step Step1;
extern float lenght;
extern int speed_L ,speed_R,plus ;
#endif