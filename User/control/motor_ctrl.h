/**
 * @file  motor_ctrl.h
 * @brief 电机闭环控制（对应 F4 的 dji_pid.c + mytim.c 里 TIM3 中断那段）。
 *
 * 单位约定：
 *   - 对外接口全部是【输出轴】单位：速度 rpm，角度 °（已经考虑减速比）
 *   - PID 内部仍是 F4 的【转子】单位：速度环反馈 = 转子 rps（speed_rpm/60），
 *     位置环反馈 = 转子累计角度 °。进出时按减速比换算，所以 F4 的参数直接能用。
 *
 * 调用 motor_ctrl_update() 的周期必须是 1ms（F4 的参数是按 1kHz 调的）。
 */
#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "pid.h"
#include <stdint.h>

typedef enum {
    MOTOR_MODE_OFF = 0,  /**< 不控制，不发控制帧 */
    MOTOR_MODE_OPEN,     /**< 直接给电流/电压 */
    MOTOR_MODE_SPEED,    /**< 速度环 */
    MOTOR_MODE_POS,      /**< 位置环串速度环 */
} motor_mode_t;

/** 一个电机的两套 PID 参数（转子单位，和 F4 一样） */
typedef struct {
    pid_param_t speed;   /**< 速度环：输入转子 rps，输出电流/电压 */
    pid_param_t pos;     /**< 位置环：输入转子 °，输出转子 rps；max_out 是默认限速 */
} motor_pid_param_t;

void motor_ctrl_init(void);

/** 1ms 调一次：算 PID 并发出控制帧 */
void motor_ctrl_update(void);

/** 关闭电机（先发一帧 0 再停发） */
void motor_ctrl_off(int idx);

/** 开环：直接给电流（3508/2006）或电压（6020） */
void motor_ctrl_set_open(int idx, int16_t out);

/** 速度模式，目标为输出轴 rpm */
void motor_ctrl_set_speed(int idx, float shaft_rpm);

/**
 * 位置模式，目标为输出轴角度 °（相对上电零点）。
 * @param max_shaft_rpm 限速（输出轴 rpm），<=0 用参数里的默认值。
 *        对应 F4 PID_motoloop 里 mode 1 时 setspeed 的作用。
 */
void motor_ctrl_set_pos(int idx, float shaft_deg, float max_shaft_rpm);

/** 替换某个电机的 PID 参数（会清积分） */
void motor_ctrl_set_param(int idx, const motor_pid_param_t *param);

motor_mode_t motor_ctrl_mode(int idx);

/** 在线改 kp/ki/kd（不清积分），pos_loop = 0 速度环，1 位置环 */
void motor_ctrl_set_gains(int idx, int pos_loop, float kp, float ki, float kd);

/** 减速比（输出轴单位 <-> 转子单位换算用） */
float motor_ctrl_gear(int idx);

/** 调试用：取内部 PID 状态 */
const pid_t *motor_ctrl_speed_pid(int idx);
const pid_t *motor_ctrl_pos_pid(int idx);

#endif
