#ifndef __IMU_AHRS_H
#define __IMU_AHRS_H

#include "zf_common_headfile.h"
#include "imu660.h"           // 使用原来的 IMUdata 结构体

void IMU_AHRS_Init(void);
void IMU_AHRS_Update(void);   // 放在定时器中断中调用，5ms一次
void IMU_AHRS_ResetYaw(void);


#endif