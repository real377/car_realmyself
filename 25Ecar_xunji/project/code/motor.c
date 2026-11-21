#include "zf_common_headfile.h"
/*************************************************电机运行***************************************************/
PID Speed = {0};
Step Step1 = {0};
int speed_L = 0 ,speed_R = 0,plus = 0,speed_L1 = 0 ,speed_R1 = 0;
float lenght = 0,length_dt = 0.02;

PID PidLeft = {0};
PID PidRight = {0};

// 在 motor.c 中定义，用于 PID 反馈
int16_t speed_left_now = 0;
int16_t speed_right_now = 0;


void PID_Motor_Init(void)
{
    	Speed.kp = 5;
    	Speed.ki = 3;
		//Speed.kp = 1;
    	//Speed.ki = 0.5;
		//Speed.target_val = 35;
		Speed.target_val = 35;
	    // 左轮 PID 参数
    PidLeft.kp = 5.0f;
    PidLeft.ki = 4.0f;

    PidLeft.target_val = 25;   // 目标速度
    
    // 右轮 PID 参数（初期完全一样，如果左右机械差异大，可微调）
    PidRight.kp = 5.0f;
    PidRight.ki = 4.0f;
    PidRight.target_val = 25;
//		Speed.kp = 3.5;
//		Speed.ki = 2.8;
//		Speed.target_val = 50;
}

void Get_Speed()
{
	
		static int16 prev_L[10] = {0}, prev_R[10] = {0};
    int32 avg_L = 0, avg_R = 0;
    
    // 保存当前原始脉冲数
    int16 rawL = L_count;
    int16 rawR = R_count;
    ctimer_clean();  // 清零以便下一周期计数
    
    // 滑动平均滤波
    for (int i = 9; i > 0; i--)
    {
        prev_L[i] = prev_L[i - 1];
        prev_R[i] = prev_R[i - 1];
        avg_L += prev_L[i];
        avg_R += prev_R[i];
    }
    prev_L[0] = rawL;
    prev_R[0] = rawR;
    avg_L += rawL;
    avg_R += rawR;
    
    // 左右轮各自滤波后的速度（20ms周期，直接用脉冲数作为速度单位）
    speed_left_now  = (int16)((avg_L + 3) >> 3);
    speed_right_now = (int16)((avg_R + 3) >> 3);
	
	
		
		static int16 previousSpeed_L[10] = {0}, previousSpeed_R[10] = {0};
		float lenght_1;
		int16 averageSpeed = 0;
		uint8 i;
		speed_L = L_count;
		speed_R = R_count;
		
//		if(plus_flag==1)
//			plus += speed_L + speed_R; 
//		ceshi=plus;
//		if(plus>680)  
//		{
//			plus_flag=0;
//			zhijiao_flag=1;
//			plus=0;
//		}

		ctimer_clean();
    for (i = 9; i > 0; i--)
    {
        previousSpeed_L[i] = previousSpeed_L[i - 1];
        previousSpeed_R[i] = previousSpeed_R[i - 1];
        averageSpeed += (previousSpeed_L[i] + previousSpeed_R[i]);
    }

    previousSpeed_L[0] = speed_L;
    previousSpeed_R[0] = speed_R;
    averageSpeed += (speed_L + speed_R);
    
    
    Speed.now = (int16)((averageSpeed + 3) >> 3);//滑动滤波
	ceshi=Speed.now;
    lenght_1 = Speed.now * PulseMPS  * length_dt ; //里程
    
    Step1.Lenth = (Step1.flag & 0x01) ? (Step1.Lenth + lenght_1) : 0; //条件里程

    //lenght += lenght_1; //总里程
	if(plus_flag==1)
	{
		lenght += lenght_1; //总里程
	}
	if(lenght>0.38)  
	{
		plus_flag=0;
		zhijiao_flag=1;
		//lenght_1=0;
		lenght=0;
	}
    
    
}

static void Motor_L(int32 dutyL)//电机左轮转动
{
    if(dutyL > 0)//正转
    {
        gpio_set_level(DIR_L1,0);
        gpio_set_level(DIR_L2,1);			
        pwm_set_duty(PWM_L, dutyL );
    }
    else  //反转
    {
        gpio_set_level(DIR_L1,1);
        gpio_set_level(DIR_L2,0);		
        pwm_set_duty(PWM_L, -dutyL );
    }
}

static void Motor_R(int32 dutyR)//电机右轮转动
{
    if(dutyR > 0)//正转
    {
        gpio_set_level(DIR_R1,0);
        gpio_set_level(DIR_R2,1);		
        pwm_set_duty(PWM_R, dutyR );
    }
    else  //反转
    {
        gpio_set_level(DIR_R1,1);
        gpio_set_level(DIR_R2,0);	
        pwm_set_duty(PWM_R, -dutyR );
    }
}

void out_pwm(int32 dutyL, int32 dutyR)//左 右
{
	dutyR = func_limit_ab(dutyR, -8000, 8000);
    dutyL = func_limit_ab(dutyL, -8000, 8000);
    Motor_R(-dutyR);
    Motor_L(-dutyL);
}
void PID_motor(PID *pid)
{
    pid->ek1 = pid->ek;
    pid->ek = pid->target_val - pid->now;
    pid->pid_delta = (pid->kp * (pid->ek - pid->ek1) + pid->ki * pid->ek);
    pid->output_val += (int16) pid->pid_delta;
    pid->output_val = (int16)(0.8* pid->output_val + 0.2 * pid->output_val_last);
    pid->output_val = func_limit_ab(pid->output_val,-7000,7000);
    pid->output_val_last = pid->output_val;
}


// ★ 新增：根据传入速度动态设定目标值
void motor_turn_with_speed(int16 target_speed)
{
    PidLeft.target_val = target_speed;
    PidRight.target_val = target_speed;
    Motor_Speed_Control();
}



void motor_turn()
{
    // 目标速度可以在这里动态调整
    PidLeft.target_val = 15;
    PidRight.target_val = 15;	
	
//		price.out_ki = 0.3;
			
    Motor_Speed_Control();
//			out_pwm(Speed.output_val,Speed.output_val);	
}

void zhijiao_turn()
{
		Speed.target_val = 10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val +1000  ,Speed.output_val -1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
}
void plus_turn()
{
	out_pwm(Speed.output_val   ,Speed.output_val  );
}

void line()
{
		if(price.output_val > 4000)price.output_val = 4000;

		out_pwm(1500 - price.output_val*0.5,1500 + price.output_val*0.5);
}

void stop()
{
				out_pwm(0,0);
		    gpio_set_level(DIR_R1,0);
        gpio_set_level(DIR_R2,0);	
			  gpio_set_level(DIR_L1,0);
        gpio_set_level(DIR_L2,0);	

	
}
int32 dutyL,dutyR;

void Motor_Speed_Control(void)
{
    // 更新左右轮当前速度反馈
    PidLeft.now = speed_left_now;
    PidRight.now = speed_right_now;
    
    // 分别计算左轮和右轮的 PID（需要修改 PID_motor 使其不依赖全局变量）
    PID_motor(&PidLeft);
    PID_motor(&PidRight);
    
	
    // 结合方向环输出 price.output_val，注意极性
		if(PidLeft.output_val > 3300)PidLeft.output_val = 3600;
		if(PidRight.output_val > 3300)PidRight.output_val = 3600;
	
		if(price.output_val > 500)price.output_val = 500;

	
    dutyL = (int32)PidLeft.output_val/2 + (int32)price.output_val;
    dutyR = (int32)PidRight.output_val/2 - (int32)price.output_val;
	
	
	
    out_pwm(dutyL, dutyR);
}

void Motor_Speed_Control111(void)
{
    // 更新左右轮当前速度反馈
    PidLeft.now = speed_left_now;
    PidRight.now = speed_right_now;
    
    // 分别计算左轮和右轮的 PID（需要修改 PID_motor 使其不依赖全局变量）
    PID_motor(&PidLeft);
    PID_motor(&PidRight);
    
	
    // 结合方向环输出 price.output_val，注意极性
		if(PidLeft.output_val > 3300)PidLeft.output_val = 3500;
		if(PidRight.output_val > 3300)PidRight.output_val = 3500;
	
		price.output_val = 0;

	
    dutyL = (int32)PidLeft.output_val/2 + (int32)price.output_val;
    dutyR = (int32)PidRight.output_val/2 - (int32)price.output_val;
	
	
	
    out_pwm(dutyL, dutyR);
}


