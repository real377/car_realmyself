/**转向环，分为内环和外环，内环阻尼作用，为了防止车行驶时左摇右摆
                           外环转向作用，可以使得车沿着电磁线跑
													 外环周期>内环周期
**/

//增大P,降低阻尼内切
#include "zf_common_headfile.h"
#define FILTER_Turn_0 0.8f
#define FILTER_Turn_1 0.1f
#define FILTER_Turn_2 0.07f
#define FILTER_Turn_3 0.03f
turn price = {0};
void DerectPid_Init()
{
    price.out_ki = 0.1;			//类似积分作用，消除静差但影响响应速度  外环
    price.kp = 1.1;				//偏差越大，输出控制量越大    外环，航向角
    price.kd = 1.15;				//值越大，系统越稳定，但响应会变慢
	
    price.in_kp = 110;//25		//值越大，新数据影响力越强，响应快但噪声敏感   内环，角速度
    price.in_kd = 20;			//值越大，系统越平滑，但响应延迟增加
	
//	price.out_ki = 1;			//类似积分作用，消除静差但影响响应速度
//    price.kp = 0.95;				//偏差越大，输出控制量越大
//    price.kd = 1.06;				//值越大，系统越稳定，但响应会变慢
//	
//    price.in_kp = 125;//25		//值越大，新数据影响力越强，响应快但噪声敏感
//    price.in_kd = 0;			//值越大，系统越平滑，但响应延迟增加
}
static float Turn_Out_Filter(float turn_out)
{
    static float Pre1_Error[4] = {0.0f};
    Pre1_Error[3] = Pre1_Error[2];
    Pre1_Error[2] = Pre1_Error[1];
    Pre1_Error[1] = Pre1_Error[0];
    Pre1_Error[0] = turn_out;
    return (Pre1_Error[0] * FILTER_Turn_0) +
           (Pre1_Error[1] * FILTER_Turn_1) +
           (Pre1_Error[2] * FILTER_Turn_2) +
           (Pre1_Error[3] * FILTER_Turn_3);
}
/**********转向环控制**********/
////转向外环20ms,转向内环10ms
void DifferControl()
{ 
    if(price.COUNTS == 2)
    {
        price.COUNTS = 0;
		DerectError_Get();
    }
    price.COUNTS++;
    price.target_val = price.target_val_last

                       + (price.target_val - price.target_val_last) * price.COUNTS / 2;
    Direction_out();
}

void DerectError_Get(void)  //外环1
{
//		IMUdata.flag =1;
		//price.in_ek = 45 - IMUdata.Yaw;
		//price.in_ek = 0 - IMUdata.Yaw;
	
    price.target_val =   price.in_kp  * price.in_ek +
                        price.in_kd * (price.in_ek - price.ek2);
//	price.target_val = 0;//开启这句话，表转向环只有内环，外环输出为0
    price.ek2 = price.in_ek;
    price.target_val_last = price.target_val;
}

//void DerectError_Get2(void)  //外环2(角度环）
//{
//	uint8_t first_run = 1;
//    float initial_yaw = 0;  // 保存初始方向
////    IMUdata.flag = 1;
//    // 第一次运行时保存初始方向作为直线基准
//    if(first_run) {
//        initial_yaw = IMUdata.Yaw;
//		second_yaw = initial_yaw;
//        first_run = 0;
//		second_run = 1;
//    }
//    // 角度误差：保持初始方向
//    price.in_ek = initial_yaw - IMUdata.Yaw;
//    price.target_val =   price.in_kp  * price.in_ek +
//                        price.in_kd * (price.in_ek - price.ek2);
////		price.target_val = 0;//开启这句话，表转向环只有内环，外环输出为0
//    price.ek2 = price.in_ek;
//    price.target_val_last = price.target_val;
//}

//void DerectError_Get3(void)  //外环3(角度环）
//{
//    price.in_ek = second_yaw-IMUdata.Yaw+180;
//    price.target_val =   price.in_kp  * price.in_ek +
//                        price.in_kd * (price.in_ek - price.ek2);
////		price.target_val = 0;//开启这句话，表转向环只有内环，外环输出为0
//    price.ek2 = price.in_ek;
//    price.target_val_last = price.target_val;
//}

/**
 * @file		方向内环
 * @note      	        串级
 * @brief		STC16
 * @author		Laplace
 * @date		2021年11月25日
 */
void Direction_out(void)
{

    price.ek = price.target_val 

               - IMUdata.GYRO_Z * price.out_ki;

    price.output_val = price.kp * price.ek

                       + price.kd * (price.ek - price.ek1);
    price.ek1 = price.ek;//保存上次误差

    price.output_val = Turn_Out_Filter(price.output_val);
    price.output_val_last = price.output_val;
}