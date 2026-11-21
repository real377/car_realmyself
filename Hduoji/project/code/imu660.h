#ifndef __IMU660_H_
#define __IMU660_H_


//ICM数据
typedef struct
    {

    float GYRO_Z;
    //角度
    float Pitch;
	float Yaw;
	float Yaw_offset;	
	float Yaw_better;
	float first;
	float second;
    //Y轴角速度

    float YawVelocity;
    float YawVelocity_offset_1,
          YawVelocity_offset_2;//偏航角速度偏置
    
    uint8 flag;//icm积分标志位
	uint8 flag_2;
	float drift_compensation;  // 动态漂移补偿值
    uint8 is_stationary;       // 是否静止标志
    uint32 stationary_start;   // 静止开始时间
	
}IMU660RB;


void ICM_OneOrderFilter(void);
void GyroOffsets(uint16 count_g);
extern IMU660RB IMUdata; 
#endif