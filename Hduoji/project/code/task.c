#include "zf_common_headfile.h"
int16 ceshi = 0;
int flag = 0;
int count_4 = 0;
float ceshi_2=0.0;
float ceshi_3=0.0;
int16 ceshi_4=0;
int zhijiao_flag = 0;
int shibie_flag = 0;
int plus_flag = 0;
int Flag_A = 0;
int Flag_B = 0;
extern int count_stop;
extern int back_go;
extern int k230_1;
extern int k230_2;
int k230_3=0;
int k230_4=0;
int k230_2zhuan=0;
int k230_flag=0;
int k230_ke1=0;
extern int key;
void buzzer(uint8_t mode)
{
	static uint8_t buzzer_states = 0;
	 if((buzzer_states & (1 << mode)) == 0)
	{
        gpio_init(A29, GPO, 0, GPO_PUSH_PULL);
        gpio_set_level(A29, 0);
        system_delay_ms(100);
        gpio_set_level(A29, 1);
        buzzer_states |= (1 << mode);  // 设置对应模式的bit
	}
}
void buzzer_2()
{
    gpio_init(A29, GPO, 0, GPO_PUSH_PULL);
    gpio_set_level(A29, 0);    // 蜂鸣器响
    system_delay_ms(100);      // 响100ms
    gpio_set_level(A29, 1);    // 蜂鸣器关闭
}
uint8_t read_line_sensors()			//检测到黑线
{ 
	uint8_t sensor_value = 0;
//	get_state();
	if(list[0] == 0)
	{sensor_value = 1;}
	if(list[1] == 0)
	{sensor_value = 1;}
	if(list[2] == 0)
	{sensor_value = 1;}
	if(list[3] == 0)
	{sensor_value = 1;}
	if(list[4] == 0)
	{sensor_value = 1;}
	if(list[5] == 0)
	{sensor_value = 1;}
	if(list[6] == 0)
	{sensor_value = 1;}
return sensor_value;
}
uint8_t  read_line_sensors2()			//没有检测到黑线
{
	uint8_t flag_1 = 0;
	if(list[0]&&list[1]&&list[2]&&list[3]&&list[4]&&list[5]&&list[6])
	{
		flag_1 = 1;
	}	
	return flag_1;
}
void task1()
{
	if(flag == 0)
	{
		IMUdata.flag =1;
		price.in_ek = IMUdata.first - IMUdata.Yaw;
		if( read_line_sensors() ==1)
		{
			buzzer(1);
			Speed.target_val = 0;
			ceshi =1;
		}
	}
}
void task2()
{
	IMUdata.flag =1;
	if(flag == 0)
	{
		price.in_ek = 0 - IMUdata.Yaw;
		if( read_line_sensors()==1)
		{
			buzzer(1);
			flag =1;
		}
	}
	if(flag == 1)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(2);
			flag = 2;
		}
	}
	if(flag == 2)
	{
		price.in_ek = 177 - IMUdata.Yaw;	//速度50
//		price.in_ek = 180 - IMUdata.Yaw;  	//速度35
		if( read_line_sensors() ==1)
		{
			buzzer(3);
			flag = 3;
		}
	}
	if(flag == 3)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(4);
			Speed.target_val = 0;
		}
	}
}
void task3_2()
{
	IMUdata.flag =1;
	if(flag == 0)
	{
		price.in_ek = 0 - IMUdata.Yaw;
		if( read_line_sensors()==1)
		{
			buzzer(1);
			flag =1;
		}
	}
	if(flag == 1)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(2);
			flag = 2;
		}
	}
	if(flag == 2)
	{
		price.in_ek = -248 - IMUdata.Yaw;
		if( read_line_sensors() ==1)
		{
			buzzer(3);
			flag = 3;
		}
	}
	if(flag == 3)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(4);
			flag =4;
			Speed.target_val = 0;
		}
	}
}
void task3()
{
	IMUdata.flag =1;
	if(flag == 0)
	{
		price.in_ek = 0 - IMUdata.Yaw;
		if( read_line_sensors()==1)
		{
			buzzer(1);
			flag =1;
		}
	}
	if(flag == 1)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(2);
			flag = 2;
		}
	}
	if(flag == 2)
	{
		price.in_ek = 212 - IMUdata.Yaw;
		if( read_line_sensors() ==1)
		{
			buzzer(3);
			flag = 3;
		}
	}
	if(flag == 3)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			buzzer(4);
			flag =4;
		}
	}
	if(flag == 4)
	{
		price.in_ek = -35 - IMUdata.Yaw;
		if( read_line_sensors() ==1)
		{
			buzzer(5);
			Speed.target_val = 0;
		}
	}
}
void task4(void)
{
	IMUdata.flag =1;
	if(flag == 0)
	{
		price.in_ek = 0 - IMUdata.Yaw;
		if( read_line_sensors()==1)
		{
			//buzzer_2();
			flag =1;
		}
	}
	if(flag == 1)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			//buzzer_2();
			flag = 2;
		}
	}
	if(flag == 2)
	{
		price.in_ek = -248 - IMUdata.Yaw;
		if( read_line_sensors() ==1)
		{
			//buzzer_2();
			flag = 3;
		}
	}
	if(flag == 3)
	{
		huidu_err();
		if(read_line_sensors2() ==1)
		{		
			//buzzer_2();
			count_4++;
			flag =0;
		}
	}
	if(count_4 == 4)
	{
		flag = 4;
		Speed.target_val = 0;
		ceshi =1;
	}
}
void zhijiao(void)
{
	int i;
	i=list[0]+list[1]+list[2]+list[3]+list[4]+list[5]+list[6];
	if(i<=4) plus_flag=1;
	//if(i==7) plus_flag=1;
	ceshi=i;
}

void task1_ti(void)
{
	zhijiao();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
//		else  
//			price.output_val=0;
		motor_turn();
	}
	else 
		zhijiao_turn();	
	//if(zhijiao_flag==1&&IMUdata.Yaw_better>35.0f&&Flag_A==0 )
	if(zhijiao_flag==1&&IMUdata.Yaw>75.0f )
	{	
		zhijiao_flag=0;
		count_4++;
	}
	//else if(zhijiao_flag==1&& IMUdata.Yaw_better>80.0f&&Flag_A==1)
//	else if(zhijiao_flag==1&& IMUdata.Yaw_better>140.0f&&Flag_A==1)
//	{	
//		zhijiao_flag=0;
//		Flag_A=2;
//	}
//	//else if(zhijiao_flag==1&& IMUdata.Yaw_better>125.0f&&Flag_A==2)
//	else if(zhijiao_flag==1&& IMUdata.Yaw_better>230.0f&&Flag_A==2)	
//	{	
//		zhijiao_flag=0;
//		Flag_A=3;
//	}
//	else if(zhijiao_flag==1&& IMUdata.Yaw_better>300.0f&&Flag_A==3)
//	{	
//		zhijiao_flag=0;
//		IMUdata.Yaw_offset = IMUdata.Yaw_better;  // 记录当前 Yaw 值作为偏移量
//		Flag_A=0;
//	}
	
}

void stop_car()
{
	int i;
	i=list[0]+list[1]+list[2]+list[3]+list[4]+list[5]+list[6];
	if(i<=4) plus_flag=1;
}

void task2_ti(void)
{
//	IMUdata.flag =1;
//	if(zhijiao_flag==0)
//	  price.in_ek = IMUdata.first - IMUdata.Yaw;
//	licheng();
//	price.out_ki = 0.5;
//    out_pwm(Speed.output_val +  price.output_val+out_go,Speed.output_val - price.output_val-out_go);
	
	//zhijiao();
  if(back_go==0)	
  {
	licheng();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
		zhijiao_turn();	
	if(zhijiao_flag==1&&IMUdata.Yaw>87.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
	}
  }	
  else if(back_go==1)
  {
	licheng_2();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
	        price.in_ek = IMUdata.first - IMUdata.Yaw;
			//huidu_err();
		motor_turn_2();
	}
	else  
	   zhijiao_turn_3();
	  //zhijiao_turn_2();	//倒车
/*	if(zhijiao_flag==1&&IMUdata.Yaw<-90.0f )  //倒车
	{	
		//plus_flag=0;
		//zhijiao_flag=0;
		count_4=1;
	}
    if(count_4)	
	{		
	    price.in_ek = -91 - IMUdata.Yaw;
		motor_turn_2();
	}*/
	if(zhijiao_flag==1&&IMUdata.Yaw>=86.0f )  
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
  }
  else if(back_go==2)  //中端   远端
  {
	 licheng_3();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
		zhijiao_turn_4();		
    if(zhijiao_flag==1&&IMUdata.Yaw<=-85.0f )  
	{	
		plus_flag=0;
		zhijiao_flag=0;
		count_4=1;
	}
  }
}

void task_A_ti(void)   //近端
{
  if(back_go==0)	//前进
  {
	licheng_A();
   if(key==1)
   {

	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
	{
	   if(k230_ke1==2)
		{
		  zhijiao_turn();
		  k230_3=2;
		}
		else if(k230_ke1==1)
		{
			zhijiao_turn_zuozhuan();
			k230_3=1;
		}
	}		
	if(zhijiao_flag==1&&IMUdata.Yaw>=86.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw<=-87.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
	}
   }
  }	
  else if(back_go==1)	//倒车
  {
	licheng_back();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
	        price.in_ek = IMUdata.first - IMUdata.Yaw;
		motor_turn_2();
	}
	else 
	{		
	  if(k230_3==2)
	      zhijiao_turn_3();
		else if(k230_3==1)
		  zhijiao_turn_2();
	}
	if(zhijiao_flag==1&&IMUdata.Yaw>=86.0f ) 
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw<=-86.0f ) 
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
  } 
  else if(back_go==2)  //回来
  {
	  licheng_A_2();
	  huidu_err();
	  motor_turn();
  }
}
void task_B_ti(void)	//中端
{
  if(back_go==0)	
  {
	licheng_B();
   if(key==1)
   {
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
	{
		if(k230_2==2)
		{
		  zhijiao_turn();
		  k230_3=2;
		}
		else if(k230_2==1)
		{
			zhijiao_turn_zuozhuan();
			k230_3=1;
		}
	}	
   }	
	if(zhijiao_flag==1&&IMUdata.Yaw>=87.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
	}
    if(zhijiao_flag==1&&IMUdata.Yaw<=-87.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
	}
  }	
  else if(back_go==1)
  {
	licheng_back();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
	        price.in_ek = IMUdata.first - IMUdata.Yaw;
		motor_turn_2();
	}
	else  
	{
		if(k230_3==2)
	      zhijiao_turn_3();
		else if(k230_3==1)
		  zhijiao_turn_2();
	}
	if(zhijiao_flag==1&&IMUdata.Yaw>=86.0f )  
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw<=-85.0f )  
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
  }
  else if(back_go==2)  
  {
	 licheng_B_2();
	 huidu_err();
	 motor_turn();
  }
}
void task_C_ti(void)  //远端
{
  if(back_go==0)	
  {
	licheng_C();
   if(key==1)
   {
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
	{
		if(k230_2zhuan==0)
		{
		  if(k230_2==2)
		  {
		    zhijiao_turn();
		    k230_3=2;
//		    k230_2zhuan=1;
		  }
		  else if(k230_2==1)
		  {
			 zhijiao_turn_zuozhuan();
			 k230_3=1;
//			 k230_2zhuan=1;
		  }
	   }
	   else if(k230_2zhuan==1)
	   {
		  if(k230_2==2)
		  {
		    zhijiao_turn();
		    k230_4=2;
//		    k230_2zhuan=2;
		  }
		  else if(k230_2==1)
		  {
			 zhijiao_turn_zuozhuan();
			 k230_4=1;
//			 k230_2zhuan=2;
		   }
		}
	}		
	if(zhijiao_flag==1&&IMUdata.Yaw>=85.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
		ceshi_2=1;
		k230_2=0;
//		k230_flag=0;
		k230_2zhuan=1;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw<=-85.0f )
	{	
		plus_flag=0;
		zhijiao_flag=0;
		ceshi_2=1;
		k230_2=0;
//		k230_flag=0;
		k230_2zhuan=1;
	}
   }
  }	
  else if(back_go==1)
  {
	licheng_C_back();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0)
	        price.in_ek = 0 - IMUdata.Yaw;
		motor_turn_2();
	}
	else  
	{
		if(k230_4==2)
	      zhijiao_turn_3();
		else if(k230_4==1)
		  zhijiao_turn_2();
	}
	if(zhijiao_flag==1&&IMUdata.Yaw>=80.0f )  
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw<=-80.0f )  
	{	
		back_go=2;
		plus_flag=0;
		zhijiao_flag=0;
	}
  }
  else if(back_go==2)  
  {
	 licheng_C_2();
	if(zhijiao_flag==0)
	{
		if(plus_flag==0 && shibie_flag==0)
			huidu_err();
		motor_turn();
	}
	else 
	{
		if(k230_3==2)
	      zhijiao_turn_4();
		else if(k230_3==1)
		  zhijiao_turn();	
	}		
    if(zhijiao_flag==1&&IMUdata.Yaw<=-83.0f )  
	{	
		plus_flag=0;
		zhijiao_flag=0;
		count_4=1;
	}
	if(zhijiao_flag==1&&IMUdata.Yaw>=83.0f )  
	{	
		plus_flag=0;
		zhijiao_flag=0;
		count_4=1;
	}
  }
}


