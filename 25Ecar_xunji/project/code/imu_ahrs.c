#include "imu_ahrs.h"
#include "math.h"

#define M_PI          3.1415926f
#define DELTA_T       0.005f      // 5ms 调用周期
#define BUFFER_SIZE   8           // 滑动平均窗口

// ----- 四元数及互补滤波变量 -----
static struct {
    float q0, q1, q2, q3;
} Q_info = {1.0f, 0.0f, 0.0f, 0.0f};

static float I_ex = 0.0f, I_ey = 0.0f;
static const float param_Kp = 1.0f;
static const float param_Ki = 0.2f;

// ----- 陀螺仪零偏 -----
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

// ----- 滑动平均缓冲区 -----
static float acc_buf[3][BUFFER_SIZE] = {0};
static float gyro_buf[3][BUFFER_SIZE] = {0};
static uint8_t buf_idx = 0;

// ----- 航向偏移 -----
static float yaw_offset_angle = 0.0f;
static uint8_t prev_flag = 1;

// ----- 上电预热与校准状态机 -----
static uint8_t  calibration_phase = 0;   // 0=预热, 1=采样, 2=完成
static uint32_t warmup_counter  = 0;     // 预热计数器
static uint32_t sample_counter  = 0;     // 采样计数器
static float    sample_sum_x = 0;
static float    sample_sum_y = 0;
static float    sample_sum_z = 0;
#define  WARMUP_MS      800            // 预热时间 3 秒
#define  SAMPLE_COUNT   200             // 采样点数（500次 × 5ms = 2.5秒）

// 快速平方根倒数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void sliding_average(float *acc_x, float *acc_y, float *acc_z,
                            float *gyro_x, float *gyro_y, float *gyro_z)
{
    acc_buf[0][buf_idx] = (float)imu660rb_acc_x;
    acc_buf[1][buf_idx] = (float)imu660rb_acc_y;
    acc_buf[2][buf_idx] = (float)imu660rb_acc_z;
    gyro_buf[0][buf_idx] = (float)imu660rb_gyro_x - gyro_offset_x;
    gyro_buf[1][buf_idx] = (float)imu660rb_gyro_y - gyro_offset_y;
    gyro_buf[2][buf_idx] = (float)imu660rb_gyro_z - gyro_offset_z;

    buf_idx = (buf_idx + 1) % BUFFER_SIZE;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        sum_ax += acc_buf[0][i];
        sum_ay += acc_buf[1][i];
        sum_az += acc_buf[2][i];
        sum_gx += gyro_buf[0][i];
        sum_gy += gyro_buf[1][i];
        sum_gz += gyro_buf[2][i];
    }

    *acc_x = sum_ax / BUFFER_SIZE;
    *acc_y = sum_ay / BUFFER_SIZE;
    *acc_z = sum_az / BUFFER_SIZE;
    *gyro_x = sum_gx / BUFFER_SIZE;   // 仍为 LSB
    *gyro_y = sum_gy / BUFFER_SIZE;
    *gyro_z = sum_gz / BUFFER_SIZE;
}

static void MahonyAHRSupdate(float gx, float gy, float gz,
                             float ax, float ay, float az)
{
    float halfT = 0.5f * DELTA_T;
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;

    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;

    I_ex += DELTA_T * ex;
    I_ey += DELTA_T * ey;
    gx += param_Kp * ex + param_Ki * I_ex;
    gy += param_Kp * ey + param_Ki * I_ey;

    q0 += (-q1*gx - q2*gy - q3*gz) * halfT;
    q1 += ( q0*gx + q2*gz - q3*gy) * halfT;
    q2 += ( q0*gy - q1*gz + q3*gx) * halfT;
    q3 += ( q0*gz + q1*gy - q2*gx) * halfT;

    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

// 上电延迟预热 + 静止确认后再校准
void IMU_AHRS_Init(void)
{
    // ----- 预填充缓冲区（使用当前瞬时值）-----
    imu660rb_get_acc();
    imu660rb_get_gyro();
    float ax = (float)imu660rb_acc_x;
    float ay = (float)imu660rb_acc_y;
    float az = (float)imu660rb_acc_z;
    float gx = (float)imu660rb_gyro_x - gyro_offset_x;
    float gy = (float)imu660rb_gyro_y - gyro_offset_y;
    float gz = (float)imu660rb_gyro_z - gyro_offset_z;

    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        acc_buf[0][i] = ax;
        acc_buf[1][i] = ay;
        acc_buf[2][i] = az;
        gyro_buf[0][i] = gx;
        gyro_buf[1][i] = gy;
        gyro_buf[2][i] = gz;
    }

    // 校准将在 IMU_AHRS_Update 的前几次调用中自动执行
    // 因为此时零偏还是0，不影响预热
    calibration_phase = 0;
    warmup_counter = 0;
    sample_counter = 0;
    sample_sum_x = 0;
    sample_sum_y = 0;
    sample_sum_z = 0;

    prev_flag = IMUdata.flag;
}

void IMU_AHRS_Update(void)
{
    // ----- 如果校准未完成，则先执行预热 → 采样流程 -----
    if (calibration_phase != 2)
    {
        imu660rb_get_gyro();   // 只读取角速度，加速度可稍后再读

        // 阶段0：预热（等待 WARMUP_MS 毫秒）
        if (calibration_phase == 0)
        {
            warmup_counter++;
            if (warmup_counter * DELTA_T * 1000 >= WARMUP_MS) {
                calibration_phase = 1;
                sample_counter = 0;
                sample_sum_x = 0;
                sample_sum_y = 0;
                sample_sum_z = 0;
            }
            // 预热期间不更新姿态，直接返回（但IMUdata.Yaw保持0）
            return;
        }

        // 阶段1：采样（采集 SAMPLE_COUNT 个点）
        if (calibration_phase == 1)
        {
            sample_sum_x += imu660rb_gyro_x;
            sample_sum_y += imu660rb_gyro_y;
            sample_sum_z += imu660rb_gyro_z;
            sample_counter++;

            if (sample_counter >= SAMPLE_COUNT) {
                gyro_offset_x = sample_sum_x / (float)SAMPLE_COUNT;
                gyro_offset_y = sample_sum_y / (float)SAMPLE_COUNT;
                gyro_offset_z = sample_sum_z / (float)SAMPLE_COUNT;
                calibration_phase = 2;   // 校准完成
                // 重新预填充缓冲区，确保滑动平均立刻使用正确的偏移后数据
                imu660rb_get_gyro();
                float gx = (float)imu660rb_gyro_x - gyro_offset_x;
                float gy = (float)imu660rb_gyro_y - gyro_offset_y;
                float gz = (float)imu660rb_gyro_z - gyro_offset_z;
                for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
                    gyro_buf[0][i] = gx;
                    gyro_buf[1][i] = gy;
                    gyro_buf[2][i] = gz;
                }
            }
            // 采样期间也不更新姿态
            return;
        }
    }

    // ----- 正常姿态解算流程（校准完成后才会执行这里）-----
    imu660rb_get_acc();
    imu660rb_get_gyro();

    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    sliding_average(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

    // 1. 更新方向环用的 GYRO_Z（保持LSB量纲）
    static float gyro_z_filtered = 0.0f;
    float raw_gyro_z = (float)imu660rb_gyro_z - gyro_offset_z;
    gyro_z_filtered = gyro_z_filtered * 0.2f + raw_gyro_z * 0.8f;
    IMUdata.GYRO_Z = gyro_z_filtered;
    IMUdata.YawVelocity = IMUdata.GYRO_Z / 14.3f;

    // 2. 单位转换：LSB → rad/s
    #define GYRO_SENSITIVITY  16.4f
    float gyro_scale = (M_PI / 180.0f) / GYRO_SENSITIVITY;

    // 3. 运行 Mahony 互补滤波
    MahonyAHRSupdate(gyro_x * gyro_scale,
                     gyro_y * gyro_scale,
                     gyro_z * gyro_scale,
                     acc_x, acc_y, acc_z);

    // 4. 提取欧拉角
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;
    float yaw_rad = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                           -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f);
    float yaw_deg = yaw_rad * 180.0f / M_PI;

    // 5. flag 逻辑
    if (IMUdata.flag == 0) {
        IMUdata.Yaw = 0.0f;
        if (prev_flag == 1) {
            yaw_offset_angle = yaw_deg;
        }
    } else {
        if (prev_flag == 0) {
            yaw_offset_angle = yaw_deg;
        }
        float raw_yaw = yaw_deg - yaw_offset_angle;
        while (raw_yaw > 180.0f) raw_yaw -= 360.0f;
        while (raw_yaw < -180.0f) raw_yaw += 360.0f;
        IMUdata.Yaw = raw_yaw;
    }
    prev_flag = IMUdata.flag;
    IMUdata.Yaw_better = IMUdata.Yaw;
}



void IMU_AHRS_ResetYaw(void)
{
    // 读取当前四元数中的绝对航向
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;
    float yaw_rad = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3,
                           -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f);
    float yaw_deg = yaw_rad * 180.0f / M_PI;
    
    // 将当前航向记为偏移，使输出Yaw归零
    yaw_offset_angle = yaw_deg;
}