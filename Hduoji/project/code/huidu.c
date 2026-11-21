#include "zf_common_headfile.h"
uint8_t list[8] = {0};
uint8_t temp[8] = {0}; 
//float weight[7] ={-40,-25,-10,0,10,25,40};
float weight[8] ={-50,-30,-15,-5,5,15,30,50};
float err = 0;
int cross_flag = 0,T_mode = 0,T_flag = 0,traget_v = 80,a = 0;
//int mode[2] = {0};
void huidu_init()
{
		gpio_init(B24,  GPI, 0, GPI_PULL_DOWN);	//L1
		gpio_init(B20,  GPI, 0, GPI_PULL_DOWN);	//L2
		gpio_init(A22,  GPI, 0, GPI_PULL_DOWN);	//L3
	
		gpio_init(A15,  GPI, 0, GPI_PULL_DOWN);	//MID
	
		gpio_init(A16, GPI, 0, GPI_PULL_DOWN);	//R1	
		gpio_init(B17, GPI, 0, GPI_PULL_DOWN);	//R2	
		gpio_init(B19,  GPI, 0, GPI_PULL_DOWN);	//R3
		
	    gpio_init(A12,  GPI, 0, GPI_PULL_DOWN);
}
// void Cross()
//{
//		uint8 j = 0;
//		for(uint8 i = 0;i < 7;i ++)
//		{
//				if(list[i] == 1)
//					j++;
//		}
//		if(j == 7 )
//		{
//				cross_flag = 1;
//		}

//}
//void cross_deal(uint8 mode)
//{
//		if(cross_flag == 1)
//		{
//					IMUdata.flag = 1;
//				if(mode == 1)
//				{						gpio_set_level(B20,0);		

//					price.in_ek = 140 - IMUdata.Yaw;
//				}
//				else if(mode == 2)
//				{
//						gpio_set_level(B20,0);	

//					price.in_ek = - 140 - IMUdata.Yaw;
//				}

//				if(IMUdata.Yaw > 85)
//				{
//						cross_flag = 0;
//						IMUdata.flag = 0;
//					T_flag = 1;
//						gpio_set_level(B20,1);
//				}	
//				if(IMUdata.Yaw < - 85)
//				{
//						cross_flag = 0;
//						IMUdata.flag = 0;	
//						T_flag = 1;					
//						gpio_set_level(B20,1);	
//				}
//		}	
//}

void get_state()
{
//	list[0] = !gpio_get_level(B7);  //中
//	list[1] = !gpio_get_level(B6);  //左1	
//	list[2] = !gpio_get_level(B8);  //左2
//	list[3] = !gpio_get_level(B9);  //左3
//	list[4] = !gpio_get_level(A12); //右2
//	list[5] = !gpio_get_level(A13); //右1
//	list[6] = !gpio_get_level(B26); //右3
	
	list[0] = gpio_get_level(B24);  
	list[1] = gpio_get_level(B20);  	
	list[2] = gpio_get_level(A22);  
	list[3] = gpio_get_level(A15);  
	list[4] = gpio_get_level(A16); 
	list[5] = gpio_get_level(B17); 
	list[6] = gpio_get_level(B19); 
    list[7] = gpio_get_level(A12); 

}
void huidu_err()
{
    int i = 0, j = 0;
    for (i = 0; i < 7; i++)  // 修复：i < 7 而不是 i < 8
    {
        if (list[i] == 1)
        {
            temp[j] = i;
            j++;
        }
    }

    // 计算当前偏差（加权平均）
				if (j > 0)  // 有传感器检测到黑线
				{
						float sum_weight = 0;
						for (i = 0; i < j; i++)
						{
								sum_weight += weight[temp[i]];
						}
						err = sum_weight / j;  // 平均权重
				}
				else  // 无传感器检测到黑线（丢线）
				{
						err = 0;  // 可改为其他恢复策略
				}
				err = -err;
				price.in_ek = err;
	
//    for (i = 0; i < 7; i++) {
//        printf("%d", list[i]);
//		
//    }
//    printf(" -> Active: %d, err=%.1f\n", j, err);


}

//void task()
//{
//	if(flag == 0)
//	{
//		if(plus > 5600)
//		{
//				T_mode = 2;//1左拐 2右拐
//				Cross();
//				cross_deal(T_mode);
//			if(T_flag == 1)
//			{
//				flag = 1;
//				plus = 0;
//				T_flag = 0;
//			}
//		}
//	}
//		if(flag == 1  )
//		{
//			if(plus > 1000)
//			{
//					Speed.target_val = 0;
//				
//					plus = 0;			
//				flag = 2;
//				
//			}

//		}
//		else if(flag == 2)
//		{	
//			cross_flag = 2;
//			IMUdata.flag = 1;	
//			price.in_ek = -230 - IMUdata.Yaw;
//			if(IMUdata.Yaw < -190)
//			{
//			cross_flag = 0;
//			IMUdata.flag = 0;	
//			flag = 3;
//			}
//		}
//		else if(flag == 3)
//		{
//			Speed.target_val = traget_v;
//			if(plus >  5400)
//			{
//			Speed.target_val = 0;
//			plus = 0;			
//			flag = 4;	

//			}
//			
//		}
////		else if(flag == 3)
////		{
////				Speed.target_val = traget_v;
////				T_mode = 1;
////				Cross();
////				cross_deal(T_mode);
////			if(T_flag == 1)
////			{
////				T_flag = 0;
////				   flag = 4;
////			}
////		}
////		else if (flag == 4)
////		{
////				if(plus > 6400)
////				{
////					Speed.target_val = 0;
////					plus = 0;
////					//system_delay_ms(2000);
////							flag = 5;
////							
////				}

////		}
////		else if(flag == 5)
////		{

////					Speed.target_val = 0 ;
////					count1 ++;
////			
////			if(count1 > 600)
////			{
////					Speed.target_val = traget_v;	
////							T_mode = 1;
////				Cross();
////				cross_deal(T_mode);
////			if(T_flag == 1)
////			{
////				T_flag = 0;
////				   flag = 6;
////			}
////		}
////		
////		
////		}


//}
