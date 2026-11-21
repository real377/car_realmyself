#include "zf_common_headfile.h"
/*************************************************电机运行***************************************************/
PID Speed = {0};
Step Step1 = {0};
int speed_L = 0 ,speed_R = 0,plus = 0,speed_L1 = 0 ,speed_R1 = 0;
float lenght = 0,length_dt = 0.1;   //length_dt = 0.02;
float lenght_2 = 0;
int count_stop=0;
int back_go=0;
extern int16 ceshi_4;
extern int shibie_flag;
extern int k230_2;
extern int k230_flag;
void PID_Motor_Init(void)
{
    	Speed.kp = 10;
    	Speed.ki = 4;
		//Speed.kp = 1;
    	//Speed.ki = 0.5;
		//Speed.target_val = 35;
		Speed.target_val = 0;
	
//		Speed.kp = 3.5;
//		Speed.ki = 2.8;
//		Speed.target_val = 50;
}

void Get_Speed()
{
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
    lenght_2=lenght_1;
	
    Step1.Lenth = (Step1.flag & 0x01) ? (Step1.Lenth + lenght_1) : 0; //条件里程

    //lenght += lenght_1; //总里程
//	if(plus_flag==1)
//	{
//		lenght += lenght_1; //总里程
//	}
//	if(lenght>0.38)  
//	{
//		plus_flag=0;
//		zhijiao_flag=1;
//		//lenght_1=0;
//		lenght=0;
//	}
    
    
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
void motor_turn()
{
	    if((back_go==0 || back_go==2))
		{
			if(shibie_flag==0)
		          Speed.target_val = 50;
			else 
			{
				Speed.target_val = 10;
				price.in_ek = 0 - IMUdata.Yaw;
			}
		   price.out_ki = 0.3;
		   //price.in_ek = IMUdata.first - IMUdata.Yaw;
           out_pwm(Speed.output_val +  price.output_val,Speed.output_val - price.output_val);
			//out_pwm(Speed.output_val,Speed.output_val);	
		}
}

void zhijiao_turn()   //右转
{
	if(back_go==0 ||back_go==2)
	{
		Speed.target_val = 10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val +1000  ,Speed.output_val -1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
	}
}
void zhijiao_turn_zuozhuan()  //左转
{
	if(back_go==0)
	{
		Speed.target_val = 10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val -1000  ,Speed.output_val +1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
	}
}
void motor_turn_2()
{
	 if(back_go==1)
	 {
	    Speed.target_val = -50;
		price.in_ek = 0 - IMUdata.Yaw;
		price.out_ki = 0.3;
        out_pwm(Speed.output_val +  price.output_val,Speed.output_val - price.output_val);
//			out_pwm(Speed.output_val,Speed.output_val);	
	 }
}

void zhijiao_turn_2()   
{
	if(back_go==1)
	{
		Speed.target_val = -10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val -1000  ,Speed.output_val +1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
	}
}
void zhijiao_turn_3()  
{
	if(back_go==1)
	{
		Speed.target_val = -10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val +1000  ,Speed.output_val -1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
		
	}
}
void zhijiao_turn_4()
{
	  if(back_go==2)
	  {
		Speed.target_val = 10;
		price.out_ki = 1;			
        out_pwm(Speed.output_val -1000  ,Speed.output_val +1000 );
//			out_pwm(Speed.output_val,Speed.output_val);	
	  }
}

void licheng(void)
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==0)
	{
	  //if(lenght==1.6f)  //近端
	  //if(lenght==5.4f)       //中端
	  if(lenght==8.6f)     //远端  
	  {
		plus_flag=1;
	  }
	  //else if(lenght>=2.4) //近端
	  //else if(lenght>=5.6)   //中端
	  else if(lenght>=8.9f)   //远端  
	  {
		zhijiao_flag=1;
		count_stop=1; 
	  }
    }
	else if(count_stop==1)
	{
	  //if(lenght>=0.8)  //近中端  
	  if(lenght==2.1f)  //远端  
	  {
//		 Speed.target_val = 0;
//		 price.output_val=0; 
//		 out_pwm(0,0);
//		 system_delay_ms(2000);
	plus_flag=1;  //远端
		 //count_stop=2;
		 //back_go=1;   
	  }
	  if(lenght>=2.3) 
	  {
		 Speed.target_val = 0;
		 price.output_val=0; 
		 out_pwm(0,0);
		 system_delay_ms(2000);
	    zhijiao_flag=1;
		count_stop=2;
	  }
	}
	else if(count_stop==2)
	{
		if(lenght>=0.8)
		{
	      Speed.target_val = 0;
		  price.output_val=0; 
		  out_pwm(0,0);
		  system_delay_ms(2000);
		  back_go=1;  
		}
	}
}
void licheng_2(void)
{
	if(count_4==0)
	   lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	else if(count_4==1)  
		lenght +=lenght_2;
	if(count_stop==2)
	{
	  if(lenght==0.5f)   //0.6
	  {
		plus_flag=1;
	  }
	  else if(lenght<=0.3)   //0.4
	  {
		zhijiao_flag=1;
		count_stop=3; 
	  }
    }
	else if(count_stop==3)
	{
	  //if(lenght<=-2.7)   //近端
	/*  if(lenght<=-6)   //中端  倒车
	  {
		 Speed.target_val = 0;
		 price.output_val=0;
		 out_pwm(0,0);
		 count_stop=4;
		 back_go=2;
	  }   */
		
	}
}
void licheng_3(void)
{
	if(count_4==0)
	    lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_4==1)  
		lenght +=lenght_2;
   if(count_stop==3)
   {
	if(lenght==2.5f)
	{
	   plus_flag=1;
	}
	else if(lenght>=2.8)
	{
	   zhijiao_flag=1;
	   count_stop=4;
	   ceshi_4=1;
	}
	
   }
  else if(count_stop==4)
  {
	//if(lenght>=5.8)  //中端 
	if(lenght>=8.5)    //远端
	{
		 Speed.target_val = 0;
		 price.output_val=0;
		 out_pwm(0,0);
		 count_stop=5;
		 back_go=3;
	}   
   }
}

void licheng_A(void)  //近端前进
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==0)
	{
	  if(lenght==8.0f)  //近端
	  {
		plus_flag=1;
	  }
	  else if(lenght>=10.8) //近端
	  {
		zhijiao_flag=1;
		count_stop=1; 
	  }
    }
	else if(count_stop==1)
	{
	  if(lenght>=2.5)  //近中端  
	  {
		 Speed.target_val = 0;
		 price.output_val=0; 
		 out_pwm(0,0);
		 system_delay_ms(2000);
		 count_stop=2;
		 back_go=1;  
		 shibie_flag=1;	  //11
	  }
	}
}
void licheng_A_2(void)   //近端回来
{
    lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(lenght>=10)  
	{
		 Speed.target_val = 0;
		 price.output_val=0;
		 out_pwm(0,0);
		 back_go=3;
	}   	
}
void licheng_back(void)  //近中端倒车
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==2)
	{
	  if(lenght==0.5f)   //0.6
	  {
		plus_flag=1;
	  }
	  else if(lenght<=-0.9)   //0.4
	  {
		shibie_flag=0;
		zhijiao_flag=1;
		count_stop=3; 
	  }
    }
}
void licheng_B(void)    //中端前进
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==0)
	{
	  if(lenght==24.0f)       //中端
	  {
		plus_flag=1;
	  }
	  else if(lenght>=27.6)   //中端
	  {
		zhijiao_flag=1;
		count_stop=1; 
	  }
    }
	else if(count_stop==1)
	{
	  if(lenght>=2.4)  //中端   
	  {
		 Speed.target_val = 0;
		 price.output_val=0; 
		 out_pwm(0,0);
		 system_delay_ms(2000);
		 count_stop=2;
		 back_go=1; 
		 shibie_flag=1;	  //11
	  }
	}
}
void licheng_B_2(void)   //中端回来
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(lenght>=26.6)  
	{
		 Speed.target_val = 0;
		 price.output_val=0;
		 out_pwm(0,0);
		 back_go=3;
	}   
}
void licheng_C(void)   //远端前进  6  16  39  
{
	
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==0)
	{
	  if(lenght>=22 &&ceshi_4==0) 
	  {
		shibie_flag=1;
		ceshi_4=1;
	  }
	  if(shibie_flag==1)
	  {
		if(lenght>=24.0f )
		   shibie_flag=0; 
	  }
	  if(lenght>=35)   //远端  
	  {
		 count_stop=1;
	  }
    }
	else if(count_stop==1)
	{
	  if(lenght>=36&&ceshi_4==1)
	  {
		shibie_flag=1;
		ceshi_4=2;
	  }
	  if(shibie_flag==1)
	  {
		if(lenght>=40.0f )
		   shibie_flag=0; 
	  }
	  if(lenght==41.0f) 
		  plus_flag=1;
	  else if(lenght>=43.0f)   //减速到转弯的时候
	  {
		 zhijiao_flag=1;
		 count_stop=2; 
	  }
	}
	else if(count_stop==2)
	{
	  if(lenght>=7&&ceshi_4==2)
	  {
		shibie_flag=1;
		ceshi_4=3;
	  }
	  if(shibie_flag==1)
	  {
		if(lenght>=11.0f )
		   shibie_flag=0; 
	  }
	  if(lenght==11.0f)  //远端  
	  {
	    plus_flag=1;  //远端 
	  }
	  if(lenght>=11.6) 
	  {
	    zhijiao_flag=1;
		count_stop=3;
	  }
	}
	else if(count_stop==3)
	{
		if(lenght>=3)
		{
		  Speed.target_val = 0;
		  price.output_val=0; 
		  out_pwm(0,0);
		  system_delay_ms(2000);
		  back_go=1; 
          shibie_flag=1;	  //11		
		}
	}
}
void licheng_C_2(void)  //远端回来
{
	if(count_4==0)
	    lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_4==1)  
		lenght +=lenght_2;
   if(count_stop==4)
   {
	if(lenght==11.0f)
	{
	   plus_flag=1;
	}
	else if(lenght>=13.5)
	{
	   zhijiao_flag=1;
	   count_stop=5;
	}
   }
  else if(count_stop==5)
  {
	if(lenght>=40)    //远端
	{
		 Speed.target_val = 0;
		 price.output_val=0;
		 out_pwm(0,0);
		 count_stop=6;
		 back_go=3;
	}   
   }
}
void licheng_C_back(void)  //远端倒车
{
	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
	if(count_stop==3)
	{
	  if(lenght==0.5f)   
	  {
		plus_flag=1;
	  }
	  //else if(lenght<=-1.2f)   //wen
	  else if(lenght<=-1.0f)   
	  {
		shibie_flag=0;
		zhijiao_flag=1;
		count_stop=4; 
	  }
    }
}



//void licheng_C(void)   //中端前进
//{
//	lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
//	if(count_stop==0)
//	{
//	  if(lenght==5.4f)       //中端
//	  {
//		plus_flag=1;
//	  }
//	  else if(lenght>=5.6)   //中端
//	  {
//		zhijiao_flag=1;
//		count_stop=1; 
//	  }
//    }
//	else if(count_stop==1)
//	{
//	  if(lenght>=0.8)  //近中端   
//	  {
//		 Speed.target_val = 0;
//		 price.output_val=0; 
//		 out_pwm(0,0);
//		 system_delay_ms(2000);
//		 count_stop=2;
//		 back_go=1;    
//	  }
//	}
//}
//void licheng_D(void)   //中端倒车
//{
//	if(count_4==0)
//	   lenght = (zhijiao_flag) ?  0: (lenght + lenght_2);
//	else if(count_4==1)  
//		lenght +=lenght_2;
//	if(count_stop==2)
//	{
//	  if(lenght==0.6f)   //0.6
//	  {
//		plus_flag=1;
//	  }
//	  else if(lenght<=0.4)   //0.4
//	  {
//		zhijiao_flag=1;
//		count_stop=3; 
//	  }
//    }
//}


void plus_turn()
{
	out_pwm(Speed.output_val   ,Speed.output_val  );
}

