#include "zf_common_headfile.h"
IMU660RB IMUdata = {0};
//float dt = 0.1;
float dt = 0.01;
#define DEG_TO_RAD  (3.14/180)
#define RAD_TO_DEG  (180/3.14)

// 一阶惯性滤波函数，用于处理陀螺仪数据
void ICM_OneOrderFilter(void)
{
    float filtered_GYRO_Z;
	float rate;  // 当前角速度（弧度/秒）
	uint32 now = Flag_B;  // 获取当前时间（需要实现）
	
    imu660rb_get_gyro();
   // if(Car_Mode == Ramps)IMUdata.Pitch = (8U * imu660ra_gyro_y + 2U * IMUdata.Pitch) *0.1f;
    
     filtered_GYRO_Z = (imu660rb_gyro_z > 0) 
                        ? 
                       (imu660rb_gyro_z - IMUdata.YawVelocity_offset_1) 
                        : 
                       (imu660rb_gyro_z - IMUdata.YawVelocity_offset_2);
    
    
	 IMUdata.GYRO_Z = (2 * IMUdata.GYRO_Z + 8 * filtered_GYRO_Z) * 0.1f;
	//ceshi_3=IMUdata.GYRO_Z ;
     //IMUdata.YawVelocity = IMUdata.GYRO_Z * 0.00625f;
     //IMUdata.YawVelocity = IMUdata.GYRO_Z  * 0.0078f;
	IMUdata.YawVelocity = IMUdata.GYRO_Z /14.3;
    
   IMUdata.Yaw = (IMUdata.flag & 0x01) == 0 ? 0 : IMUdata.Yaw + (IMUdata.YawVelocity * dt);
   
   //IMUdata.Yaw_better = (IMUdata.flag_2 & 0x01) == 0 ? 0 : IMUdata.Yaw_better + (IMUdata.YawVelocity * dt);
	
//	IMUdata.Yaw_better= IMUdata.Yaw - IMUdata.Yaw_offset*count_4;
	
	//if(IMUdata.Yaw_better>= 180) IMUdata.Yaw_better -= 360;
    //else if(IMUdata.Yaw_better <= -180) IMUdata.Yaw_better += 360;
	/*
	// 转换为角速度（弧度/秒，因为补偿代码用弧度）
    rate = IMUdata.GYRO_Z * 0.061035f * DEG_TO_RAD;  // DEG_TO_RAD = π/180
    // ========== 动态漂移补偿 ========== 
    if(fabsf(rate) < 0.01f)  // 小于0.6度/秒（约0.01弧度/秒）
    { 
        if(!IMUdata.is_stationary) 
        { 
            IMUdata.is_stationary = 1; 
            IMUdata.stationary_start = now; 
        } 
        if(now - IMUdata.stationary_start > 50)  // 静止超过1秒
        { 
            IMUdata.drift_compensation += rate * 0.001f;  // 缓慢补偿
            // 限制补偿范围（±5度/秒 = ±0.087弧度/秒）
            if(IMUdata.drift_compensation > 0.087f) IMUdata.drift_compensation = 0.087f; 
            if(IMUdata.drift_compensation < -0.087f) IMUdata.drift_compensation = -0.087f; 
        } 
    } 
    else 
    { 
        IMUdata.is_stationary = 0; 
        // 运动时补偿值逐渐衰减
        if(IMUdata.drift_compensation > 0) IMUdata.drift_compensation -= 0.0001f; 
        else if(IMUdata.drift_compensation < 0) IMUdata.drift_compensation += 0.0001f; 
    } 
     
    // 应用漂移补偿
    rate -= IMUdata.drift_compensation; 
     
    // 转换回角度/秒用于积分
    IMUdata.YawVelocity = rate * RAD_TO_DEG;  // RAD_TO_DEG = 180/π
     
    // 积分得到角度
    IMUdata.Yaw = (IMUdata.flag & 0x01) == 0 ? 0 : IMUdata.Yaw + (IMUdata.YawVelocity * dt); 
	IMUdata.Yaw_better= IMUdata.Yaw - IMUdata.Yaw_offset;
	*/
	ceshi_2=IMUdata.Yaw_better;
}

// 陀螺仪零偏校准函数
void GyroOffsets(uint16 count_g)
{
    int16 y_1 = 0, y_2 = 0;  // 分别存储正负采样值的累加和
    uint16 count1 = 0, count2 = 0; // 正负采样值的计数器
    uint16 i;
    
    // 循环采集指定次数的陀螺仪数据
    for (i = 0; i < count_g; i++)
    {
        imu660rb_get_gyro();  // 获取陀螺仪原始数据
        system_delay_ms(1);    // 每次采样间隔1ms
        
        // 根据Z轴数据正负分别处理
        if (imu660rb_gyro_z > 0)
        {
            y_1 += imu660rb_gyro_z; // 累加正值
            count1++;               // 正值计数器增加
        }
        else
        {
            y_2 += imu660rb_gyro_z; // 累加负值
            count2++;               // 负值计数器增加
        }
    }

    // 计算正负值的平均偏移量
    IMUdata.YawVelocity_offset_1 = (float)y_1 / count1;  // 正值平均偏移
    IMUdata.YawVelocity_offset_2 = (float)y_2 / count2;  // 负值平均偏移
}
