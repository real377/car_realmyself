/**
 * @file    duoji.c
 * @brief   开环舵机控制，提供四种模式：
 *          - MODE_AUTO:   自动循环正弦轨迹小幅度往复
 *          - MODE_CAMERA: 摄像头触发 — 线性步进偏移 + 快速回中
 *          - MODE_PID:    摄像头触发 — 伪PID指数平滑偏移 + 快速回中
 *          - MODE_SWEEP:  一键扫描 — 可配置中值上下升降次数与幅度
 */

#include "zf_common_headfile.h"
#include "duoji.h"
#include <math.h>

#define TWO_PI  6.283185307f

/* ===================================================================
 *  控制器状态
 * =================================================================== */
static struct {
    uint8_t  mode;
    float    phase;
    float    amplitude;
    float    frequency;
    int16    last_pwm;
    /* CAMERA / PID 共用 */
    int16    camera_target;
    int16    camera_current;
    int16    camera_offset;
    int16    camera_step;
    int16    camera_step_back;
    float    pid_kp;
    float    pid_kp_back;
    int16    pid_max_step;
    int16    pid_max_step_back;
    /* SWEEP 模式 */
    int16    sweep_amplitude;
    int16    sweep_step;
    int16    sweep_step_back;
    uint8_t  sweep_cnt_up;
    uint8_t  sweep_cnt_down;
    uint8_t  sweep_seq;       /* 当前目标序号 */
    uint8_t  sweep_total;     /* 目标总数 */
} g_osc;

int16 dianping;
/* ===================================================================
 *  duoji_init
 * =================================================================== */
void duoji_init(void)
{
    pwm_init(SERVO_2, SERVO_FREQ, SERVO2_MID);

    g_osc.mode             = DUOJI_MODE_IDLE;
    g_osc.phase            = 0.0f;
    g_osc.amplitude        = 150.0f;
    g_osc.frequency        = 2.0f;
    g_osc.last_pwm         = SERVO2_MID;
    g_osc.camera_target    = 0;
    g_osc.camera_current   = 0;
    g_osc.camera_offset    = 150;
    g_osc.camera_step      = 12;
    g_osc.camera_step_back = 25;
    g_osc.pid_kp           = 0.20f;
    g_osc.pid_kp_back      = 0.35f;
    g_osc.pid_max_step     = 12;
    g_osc.pid_max_step_back = 25;
    g_osc.sweep_amplitude  = 400;
    g_osc.sweep_step       = 10;      /* 去程：*/   //（1） 5
    g_osc.sweep_step_back  = 6;      /* 回程：*/  //（1） 10
    g_osc.sweep_cnt_up     = 1;       //往下走
    g_osc.sweep_cnt_down   = 0;       //往上走
    g_osc.sweep_seq        = 0;
    g_osc.sweep_total      = 0;
	
//	g_osc.sweep_amplitude  = 400;   //  (1)
//    g_osc.sweep_step       = 10;      /* 去程：*/   //（1） 5
//    g_osc.sweep_step_back  = 5;      /* 回程：*/  //（1） 10
//    g_osc.sweep_cnt_up     = 1;       //往下走
//    g_osc.sweep_cnt_down   = 0;       //往上走
//    g_osc.sweep_seq        = 0;
//    g_osc.sweep_total      = 0;
}


//  - amplitude 越大摆幅越大（默认 ±200PWM）
//  - frequency 越大振荡越快（默认 2 Hz）
//  - camera_step 越大响应越快但可能抖动，建议范围 10~25
//  - camera_offset 控制偏移幅度，建议范围 50~300

// "抖动"          →减小 step / max_step
//  "反应太慢"      →增大 step / max_step / kp
//  "到位冲击大"    →CAMERA 切 PID 模式，或减小 step
//  "摆幅太小"      →增大 amplitude / camera_offset
//  "回中太慢"      →增大 camera_step_back / pid_kp_back
//  "想平滑换向"    →用 AUTO 模式（正弦）

//      .sweep_amplitude = 150,   // 升降幅度 (PWM)
//      .sweep_step      = 12,    // 升降步长 (越大越快)
//      .sweep_cnt_up    = 2,     // 中值以上升降次数 (0=不做)
//      .sweep_cnt_down  = 1,     // 中值以下升降次数 (0=不做)

//  duoji_cfg_t cfg = {
//      .sweep_amplitude = 150,
//      .sweep_step      = 8,    // 去程更慢更稳
//      .sweep_step_back = 25,   // 回程更快
//      .sweep_cnt_up    = 2,
//      .sweep_cnt_down  = 1,
//  };
  
/* ===================================================================
 *  duoji_set_mode
 * =================================================================== */
void duoji_set_mode(uint8_t mode)
{
    g_osc.mode = mode;

    if (mode == DUOJI_MODE_IDLE) {
        g_osc.phase          = 0.0f;
        g_osc.camera_target  = 0;
        g_osc.camera_current = 0;
        g_osc.sweep_seq      = 0;
        pwm_set_duty(SERVO_2, SERVO2_MID);
        g_osc.last_pwm = SERVO2_MID;
    }

    if (mode == DUOJI_MODE_SWEEP) {
        /* 初始化扫描序列：总目标数 = (上次数 + 下次数) × 2 */
        g_osc.camera_current = 0;
        g_osc.sweep_seq      = 0;
        g_osc.sweep_total    = (g_osc.sweep_cnt_up + g_osc.sweep_cnt_down) * 2;
    }
}

/* ===================================================================
 *  duoji_set_camera_cmd
 * =================================================================== */
void duoji_set_camera_cmd(uint8_t cmd)
{
    /* AUTO / SWEEP 不受摄像头指令影响，防止 pit_handler_2 误改写 camera_target */
    if (g_osc.mode == DUOJI_MODE_AUTO || g_osc.mode == DUOJI_MODE_SWEEP) {
        return;
    }

    /* MODE_DYNAMIC：摄像头指令触发单次去回扫 */
    if (g_osc.mode == DUOJI_MODE_DYNAMIC) {
        static uint8_t last_cmd = 0xFF;  /* 记录上次指令方向 */

        /* STOP：立即回中（总是执行） */
        if (cmd == DUOJI_CAMERA_STOP) {
            g_osc.camera_target  = 0;
            g_osc.camera_current = 0;
            g_osc.sweep_total    = 0;
            g_osc.sweep_seq      = 0;
            last_cmd = 0xFF;
            return;
        }

        /* 忙碌中且方向不变 → 只更新参数，不打断 */
        uint8_t busy = (g_osc.sweep_seq < g_osc.sweep_total);
        if (busy && cmd == last_cmd) {
            return;  /* 不重置，让当前 sweep 继续跑完 */
        }

        /* 空闲或方向改变 → 重新开始 */
        g_osc.camera_current = 0;
        g_osc.sweep_seq      = 0;
        last_cmd = cmd;
        if (cmd == DUOJI_CAMERA_UP) {
            g_osc.sweep_cnt_up   = 1;
            g_osc.sweep_cnt_down = 0;
            g_osc.sweep_total    = 2;
        } else if (cmd == DUOJI_CAMERA_DOWN) {
            g_osc.sweep_cnt_up   = 0;
            g_osc.sweep_cnt_down = 1;
            g_osc.sweep_total    = 2;
        }
        return;
    }

    /* 其他模式：标准行为 */
    if (cmd == DUOJI_CAMERA_UP) {
        g_osc.camera_target = g_osc.camera_offset;
    } else if (cmd == DUOJI_CAMERA_DOWN) {
        g_osc.camera_target = -g_osc.camera_offset;
    } else {
        g_osc.camera_target = 0;
    }
}
//在这里改参数
/* ===================================================================
 *  duoji_set_sweep_amp  —  DYNAMIC 模式动态调幅（摄像头 LOCK 指令用）
 * =================================================================== */
void duoji_set_sweep_amp(int16 amp)
{
//    if (amp < 50)  amp = 20;
//    if (amp > 100) amp = 800;
    g_osc.sweep_amplitude = amp;
}

/* ===================================================================
 *  duoji_set_sweep_step  —  DYNAMIC 模式动态调步长
 * =================================================================== */
void duoji_set_sweep_step(int16 out, int16 back)
{
//    if (out  < 2) out  = 2;
//    if (out  > 50) out  = 50;
//    if (back < 2) back = 2;
//    if (back > 50) back = 50;
    g_osc.sweep_step      = out;
    g_osc.sweep_step_back = back;
}

/* ===================================================================
 *  duoji_apply_config
 * =================================================================== */
void duoji_apply_config(const duoji_cfg_t *cfg)
{
    if (!cfg) return;

    g_osc.amplitude         = cfg->amplitude;
    g_osc.frequency         = cfg->frequency;
    g_osc.camera_offset     = cfg->camera_offset;
    g_osc.camera_step       = cfg->camera_step;
    g_osc.camera_step_back  = cfg->camera_step_back;
    g_osc.pid_kp            = cfg->pid_kp;
    g_osc.pid_kp_back       = cfg->pid_kp_back;
    g_osc.pid_max_step      = cfg->pid_max_step;
    g_osc.pid_max_step_back = cfg->pid_max_step_back;
    g_osc.sweep_amplitude   = cfg->sweep_amplitude;
    g_osc.sweep_step        = cfg->sweep_step;
    g_osc.sweep_step_back   = cfg->sweep_step_back;
    g_osc.sweep_cnt_up      = cfg->sweep_cnt_up;
    g_osc.sweep_cnt_down    = cfg->sweep_cnt_down;
    g_osc.phase             = 0.0f;
}

/* ===================================================================
 *  sweep_next_target  —  获取扫描序列中下一个目标偏移
 *
 *  序列: +amp, 0, +amp, 0, ... (sweep_cnt_up 次)
 *        -amp, 0, -amp, 0, ... (sweep_cnt_down 次)
 *
 *  sweep_cnt_up = 2, sweep_cnt_down = 1 →
 *    [+amp, 0, +amp, 0, -amp, 0]
 * =================================================================== */
static int16 sweep_next_target(void)
{
    uint8_t i    = g_osc.sweep_seq;
    uint8_t up_n = g_osc.sweep_cnt_up * 2;

    if (i >= up_n) {
        /* 进入下方区域 */
        i -= up_n;
        return (i & 1) ? 0 : -g_osc.sweep_amplitude;
    }
    /* 上方区域 */
    return (i & 1) ? 0 : g_osc.sweep_amplitude;
}

/* ===================================================================
 *  linear_step  —  线性步进：朝 target 走 step 步长
 * =================================================================== */
static void linear_step(int16 *current, int16 target, int16 step)
{
    int16 diff     = target - *current;
    int16 abs_diff = (diff >= 0) ? diff : (int16)(-diff);

    if (abs_diff <= step) {
        *current = target;
    } else if (diff > 0) {
        *current += step;
    } else {
        *current -= step;
    }
}

/* ===================================================================
 *  pseudo_pid_step
 * =================================================================== */
static int16 pseudo_pid_step(int16 error, float kp, int16 max_step)
{
    if (error == 0) return 0;

    int16 step = (int16)((float)error * kp);

    if (step >  max_step) step =  max_step;
    if (step < -max_step) step = -max_step;

    if (step == 0) step = (error > 0) ? 1 : -1;
    return step;
}

/* ===================================================================
 *  duoji_update  —  核心：每个控制周期调用
 * =================================================================== */
void duoji_update(uint32_t dt_ms)
{
    int16 target, offset;

    /* ================================================================
     *  MODE_AUTO — 正弦轨迹
     * ================================================================ */
    if (g_osc.mode == DUOJI_MODE_AUTO) {

        float dt_sec = (float)dt_ms * 0.001f;
        g_osc.phase += TWO_PI * g_osc.frequency * dt_sec;
        if (g_osc.phase >= TWO_PI) g_osc.phase -= TWO_PI;
        offset = (int16)(g_osc.amplitude * sinf(g_osc.phase));
        target = SERVO2_MID + offset;

    /* ================================================================
     *  MODE_CAMERA — 线性步进
     * ================================================================ */
    } else if (g_osc.mode == DUOJI_MODE_CAMERA) {

        int16 step = (g_osc.camera_target == 0) ? g_osc.camera_step_back
                                                : g_osc.camera_step;
        linear_step(&g_osc.camera_current, g_osc.camera_target, step);
        offset = g_osc.camera_current;
        target = SERVO2_MID + offset;

    /* ================================================================
     *  MODE_PID — 伪PID指数平滑
     * ================================================================ */
    } else if (g_osc.mode == DUOJI_MODE_PID) {

        int16   error  = g_osc.camera_target - g_osc.camera_current;
        uint8_t is_back = (g_osc.camera_target == 0);
        g_osc.camera_current += pseudo_pid_step(error,
            is_back ? g_osc.pid_kp_back      : g_osc.pid_kp,
            is_back ? g_osc.pid_max_step_back : g_osc.pid_max_step);
        offset = g_osc.camera_current;
        target = SERVO2_MID + offset;

    /* ================================================================
     *  MODE_SWEEP — 一键扫描序列
     *
     *  序列: 中位 → +amp → 中位 → +amp → 中位 → ... (sweep_cnt_up 次)
     *              → -amp → 中位 → -amp → 中位 → ... (sweep_cnt_down 次)
     *              → 停在原位
     *
     *  中途切换到其他模式即可打断。
     * ================================================================ */
    } else if (g_osc.mode == DUOJI_MODE_SWEEP && dianping==1) {

        if (g_osc.sweep_seq < g_osc.sweep_total) {
            g_osc.camera_target = sweep_next_target();
            int16 sw_step = (g_osc.camera_target == 0) ? g_osc.sweep_step_back
                                                       : g_osc.sweep_step;
            linear_step(&g_osc.camera_current, g_osc.camera_target, sw_step);
            if (g_osc.camera_current == g_osc.camera_target) {
                g_osc.sweep_seq++;
            }
            offset = g_osc.camera_current;
            target = SERVO2_MID + offset;
        } else {
            return;
        }

    /* ================================================================
     *  MODE_DYNAMIC — 摄像头动态触发单次去回（复用 SWEEP 序列）
     *
     *  摄像头 UP   → 去 +sweep_amplitude 再自动回中（一次完整去回）
     *  摄像头 DOWN → 去 -sweep_amplitude 再自动回中
     *  摄像头 STOP → 立即回中位
     *
     *  参数由 duoji_apply_config 或直接修改 g_osc 实时更新。
     * ================================================================ */
    } else if (g_osc.mode == DUOJI_MODE_DYNAMIC) {

        if (g_osc.sweep_seq < g_osc.sweep_total) {
            /* 获取当前目标：+amp 或 0 或 -amp */
            g_osc.camera_target = sweep_next_target();

            /* 伪PID指数平滑：步长 = error × kp，越近越慢 */
            int16 error   = g_osc.camera_target - g_osc.camera_current;
            uint8_t back  = (g_osc.camera_target == 0);       /* 回程标记 */
            float   kp    = back ? 0.35f : 0.25f;            /* 回程更快 */
            int16   max_s = back ? g_osc.sweep_step_back     /* 回程步长上限 */
                                 : g_osc.sweep_step;          /* 去程步长上限 */
            g_osc.camera_current += pseudo_pid_step(error, kp, max_s);

            /* 到达目标 → 推进到序列下一步 */
            if (g_osc.camera_current == g_osc.camera_target) {
                g_osc.sweep_seq++;
            }

            offset = g_osc.camera_current;
            target = SERVO2_MID + offset;
        } else {
            /* 序列全部完成，保持不动 */
            return;
        }

    } else {
        /* 未匹配任何模式（IDLE），不输出 */
        return;
    }

    /* 去重写入：仅 PWM 值变化时才写硬件，避免毛刺 */
    if (target != g_osc.last_pwm) {
        pwm_set_duty(SERVO_2, (uint16)target);
        g_osc.last_pwm = target;
    }
}

/* ===================================================================
 *  duoji_get_pwm_offset
 * =================================================================== */
int16 duoji_get_pwm_offset(void)
{
    if (g_osc.mode == DUOJI_MODE_CAMERA ||
        g_osc.mode == DUOJI_MODE_PID    ||
        g_osc.mode == DUOJI_MODE_DYNAMIC ||
        g_osc.mode == DUOJI_MODE_SWEEP) {
        return g_osc.camera_current;
    }
    return (int16)(g_osc.amplitude * sinf(g_osc.phase));
}

/* ===================================================================
 *  servo  —  旧版兼容
 * =================================================================== */
extern int16 pwm_ceshi;

