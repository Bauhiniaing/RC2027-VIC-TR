/**
 * @file  pid.h
 * @brief 通用位置式 PID，一个结构体只管一个环。
 *        算法和 F4 dji_pid.c 的 PID_motoloop 一致（P + 累加 I + 误差差分 D），
 *        所以 F4 的参数可以直接用。修正了 F4 的几个问题：
 *          - 积分有限幅（F4 外环的积分没有限幅）
 *          - 死区内误差按 0 处理，保持积分输出（F4 直接 return 0，输出会突变）
 *          - 提供 pid_reset()，切模式/离线时清积分
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float max_out;    /**< 输出限幅（对称） */
    float max_iout;   /**< 积分项限幅，0 表示不限 */
    float max_dout;   /**< 微分项限幅，0 表示不限（F4 外环有这个） */
    float deadband;   /**< |误差| 小于它按 0 处理 */
} pid_param_t;

typedef struct {
    pid_param_t p;
    float set, get;
    float err, last_err;
    float pout, iout, dout;
    float out;
} pid_t;

void  pid_init(pid_t *pid, const pid_param_t *param);
void  pid_reset(pid_t *pid);
float pid_calc(pid_t *pid, float set, float get);

#endif
