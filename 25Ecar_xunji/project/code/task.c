#include "zf_common_headfile.h"
int16 ceshi = 0;
int flag = 0;
int count_4 = 0;
float ceshi_2=0.0;
float ceshi_3=0.0;
int zhijiao_flag = 0;
int plus_flag = 0;
int Flag_A = 0;
int Flag_B = 0;
int count6 = 0;


// ==================== 圈数计数 ====================
int  lap_target  = 1;       // 目标圈数（按键调整 1~5）
int  lap_current = 0;       // 已完成圈数
int  corner_cnt  = 0;       // 当前圈拐角计数 0~4
int  lap_stop    = 0;       // 停车标志：1=已完成目标圈数
int  lap_running = 0;       // 是否开始计数（小车启动后置1）

// 拐角计数冷却定时器
#define CORNER_COOLDOWN_TICKS   100     // 冷却 100 tick = 1.0 秒（10ms/tick）
static int  corner_cooldown     = 0;
static int  prev_corner_state   = 0;




void buzzer(uint8_t mode)
{
	static uint8_t buzzer_states = 0;
	 if((buzzer_states & (1 << mode)) == 0)
	{
        gpio_init(A29, GPO, 0, GPO_PUSH_PULL);
        gpio_set_level(A29, 0);
        system_delay_ms(100);
        gpio_set_level(A29, 1);
        buzzer_states |= (1 << mode);
	}
}
void buzzer_2()
{
    gpio_init(A29, GPO, 0, GPO_PUSH_PULL);
    gpio_set_level(A29, 0);
    system_delay_ms(100);
    gpio_set_level(A29, 1);
}
uint8_t read_line_sensors()			// 检测到有线
{ 
	uint8_t sensor_value = 0;
	if(list[0] == 0) {sensor_value = 1;}
	if(list[1] == 0) {sensor_value = 1;}
	if(list[2] == 0) {sensor_value = 1;}
	if(list[3] == 0) {sensor_value = 1;}
	if(list[4] == 0) {sensor_value = 1;}
	if(list[5] == 0) {sensor_value = 1;}
	if(list[6] == 0) {sensor_value = 1;}
	return sensor_value;
}
uint8_t read_line_sensors2()			// 无线（全白）
{
	uint8_t flag_1 = 0;
	if(list[0]&&list[1]&&list[2]&&list[3]&&list[4]&&list[5]&&list[6])
	{
		flag_1 = 1;
	}	
	return flag_1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     拐角计数 + 定时器冷却 → 圈数判定
// 参数说明     void
// 返回参数     void
// 使用示例     放在 10ms PIT 中断中调用
// 备注信息     
//   原理：检测 corner_state 0→1 跳变（进入左拐角）
//        每次计数后启动 1 秒冷却定时器，防止同一拐角重复计数
//        4 次拐角 = 1 圈
//-------------------------------------------------------------------------------------------------------------------
void lap_counter(void)
{
    // 冷却定时器递减
    if (corner_cooldown > 0)
        corner_cooldown--;

    // ★ 上升沿检测：corner_state 从 0 跳变到 1，且冷却已过
    if (corner_state == 1 && prev_corner_state == 0 && corner_cooldown == 0)
    {
        if (lap_running)
        {
            corner_cnt++;
            corner_cooldown = CORNER_COOLDOWN_TICKS;    // 启动冷却

            // 4 个拐角 → 一圈
            if (corner_cnt >= 4)
            {
                corner_cnt = 0;
                lap_current++;
            }
        }
    }

    prev_corner_state = corner_state;

    // 达到目标圈数 → 停车
    if (lap_current >= lap_target && lap_running)
    {
        lap_stop    = 1;
        lap_running = 0;
    }
}

/* ======== 在 task.c 顶部添加状态、参数定义 ======== */
#define STRAIGHT_SPEED           50        // 直行目标速度（编码器单位）
#define ROTATE_SPEED             15        // 原地旋转目标速度（慢速）
#define STRAIGHT_DIST_TARGET     300      // 直行编码器总脉冲阈值（两轮平均）400
#define STRAIGHT_TIME_MAX        100       // 直行最大 tick 数（10ms/tick，这秒）70
#define TURN_TIME_MAX            300       // 转弯最大 tick 数（3秒）
#define TURN_ENCODER_THRESHOLD   6000       // 转弯时两轮总脉冲变化阈值



/* ======== 全局变量（task.c 文件顶部附近） ======== */
// 原有变量，不需要删
int flag1 = 0, flag2 = 0, abd = 0, count7 = 107;

// 新增状态机所需变量
CornerProcessState cp_state = PROC_NONE;
static int straight_start_tick = 0;
static int32_t straight_start_Lpulse = 0, straight_start_Rpulse = 0;
static int turn_start_tick = 0;
static int32_t turn_start_Lpulse = 0, turn_start_Rpulse = 0;

// 拐点边沿检测
static int prev_corner = 0;

int32_t avg_dist;



/* ======== 修改后的 tracking 函数 ======== */
void tracking(void)
{
    // 静态 tick 计数器（每 10ms 调用一次 tracking，所以 tick 代表时间）
    static int global_tick = 0;
    global_tick++;  // 你可以借用现有的系统 tick，这里先简单自增

    // 圈数到位则停车
    if (lap_stop) {
        stop();
        return;
    }

    // ---------- 阶段 0：正常巡线 ----------
    if (cp_state == PROC_NONE) {
        // 维持正常巡线（方向控制生效）
        DerectError_Get();
        Direction_out();
        motor_turn_with_speed(50);   // 巡线速度

        // 检测拐点上升沿（从无到有）
        if ((corner_state == 1 || corner_state == -1) && prev_corner == 0) {
            // 进入直行阶段
            cp_state = PROC_STRAIGHT;
            // 记录开始时间（tick）和编码器总脉冲
            straight_start_tick = global_tick;
            straight_start_Lpulse = L_pulse_sum;
            straight_start_Rpulse = R_pulse_sum;
        }
    }
    // ---------- 阶段 1：拐点后直行 ----------
    else if (cp_state == PROC_STRAIGHT) {
        // 直行：两轮同速，方向控制关掉，只靠速度 PID 保直线
        PidLeft.target_val  = STRAIGHT_SPEED;
        PidRight.target_val = STRAIGHT_SPEED;
        Motor_Speed_Control111();    // price.output_val 内置置 0

        // 计算经过的 tick 数
        int ticks = global_tick - straight_start_tick;
        // 计算两轮脉冲变化（取绝对值是因为可能反转，用 abs 或差值）
        int32_t dL = (L_pulse_sum > straight_start_Lpulse) ? (L_pulse_sum - straight_start_Lpulse)
                                                           : (straight_start_Lpulse - L_pulse_sum);
        int32_t dR = (R_pulse_sum > straight_start_Rpulse) ? (R_pulse_sum - straight_start_Rpulse)
                                                           : (straight_start_Rpulse - R_pulse_sum);
        avg_dist = (dL + dR) / 2;

        // 退出条件：路程达到或时间到
        if (avg_dist >= STRAIGHT_DIST_TARGET || ticks >= STRAIGHT_TIME_MAX) {
            // 进入原地旋转阶段
            cp_state = PROC_TURN;
            turn_start_tick = global_tick;
            turn_start_Lpulse = L_pulse_sum;
            turn_start_Rpulse = R_pulse_sum;
        }
    }
		
    // ---------- 阶段 2：原地缓慢转弯 ----------
    else if (cp_state == PROC_TURN) {
        // 两轮反向，低速，用速度闭环保证稳定
        // 根据拐点方向设定转向，这里假设 corner_state == 1 为左转，-1 为右转
			/*
        if (corner_state == 1) {          // 左拐 → 左轮反转，右轮正转
            PidLeft.target_val  = -ROTATE_SPEED;
            PidRight.target_val =  ROTATE_SPEED;
        } else if (corner_state == -1) { // 右拐 → 左轮正转，右轮反转
            PidLeft.target_val  =  ROTATE_SPEED;
            PidRight.target_val = -ROTATE_SPEED;
        }
        Motor_Speed_Control111();
			*/

				out_pwm(0, 1400);
			
			  gpio_set_level(DIR_L1,0);
        gpio_set_level(DIR_L2,0);	
			
			
				
        int ticks = global_tick - turn_start_tick;
        // 编码器变化量（粗略代表旋转角度）
        int32_t dL = (L_pulse_sum > turn_start_Lpulse) ? (L_pulse_sum - turn_start_Lpulse)
                                                       : (turn_start_Lpulse - L_pulse_sum);
        int32_t dR = (R_pulse_sum > turn_start_Rpulse) ? (R_pulse_sum - turn_start_Rpulse)
                                                       : (turn_start_Rpulse - R_pulse_sum);
        int32_t turn_pulse = dL + dR;   // 总脉冲变化

        // 检测是否重新看到线（中间三传感器之一）
        uint8_t line_found = list[3] && list[2] && list[4];

        // 退出条件：看到线 / 超时 / 转过足够角度
        if (!line_found )//|| ticks >= TURN_TIME_MAX || turn_pulse >= TURN_ENCODER_THRESHOLD
				{
//						stop();
				    cp_state = PROC_NONE;
            // 重置方向控制积分
            Direction_Reset();
            // 立即减速，避免惯性冲过
            PidLeft.target_val  = 0;
            PidRight.target_val = 0;
            Motor_Speed_Control111();

						
        }
				

				

    }

    // 更新边沿记录
    prev_corner = corner_state;
}





