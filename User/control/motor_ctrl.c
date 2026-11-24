#include "motor_ctrl.h"
#include "dji_motor.h"
#include "main.h"
#include <string.h>

/* ---------------- 默认参数（转子单位，来自 F4 Motor_PID_Init） ---------------- */

static const motor_pid_param_t k_default[DJI_TYPE_NUM] = {
    /* 3508：F4 里没有调过，先用 2006 的参数，上电后必须重新调！ */
    [DJI_M3508] = {
        .speed = { .kp = 1000.0f, .ki = 6.0f, .kd = 0.0f, .max_out = 16384.0f, .max_iout = 2000.0f },
        .pos   = { .kp = 0.2f, .ki = 0.0f, .kd = 0.006f, .max_out = 400.0f, .max_dout = 100.0f },
    },
    /* 2006：F4 PID_2006 = {1000, 6, 0, 0.2, 0, 0.006}，max_out = {10000, 400}，max_I_D = {2000, 100} */
    [DJI_M2006] = {
        .speed = { .kp = 1000.0f, .ki = 6.0f, .kd = 0.0f, .max_out = 10000.0f, .max_iout = 2000.0f },
        .pos   = { .kp = 0.2f, .ki = 0.0f, .kd = 0.006f, .max_out = 400.0f, .max_dout = 100.0f },
    },
    /* 6020：F4 PID_6020 = {40, 3, 0, 0.2, 0, 0}，max_out = {3000, 180}，max_I_D = {1000, 100} */
    [DJI_GM6020] = {
        .speed = { .kp = 40.0f, .ki = 3.0f, .kd = 0.0f, .max_out = 3000.0f, .max_iout = 1000.0f },
        .pos   = { .kp = 0.2f, .ki = 0.0f, .kd = 0.0f, .max_out = 180.0f, .max_dout = 100.0f },
    },
};

/* 减速比（和 dji_motor.c 保持一致） */
static float gear_of(dji_type_t t)
{
    switch (t) {
    case DJI_M3508: return 3591.0f / 187.0f;
    case DJI_M2006: return 36.0f;
    default:        return 1.0f;
    }
}

#define MAX_MOTORS 24

typedef struct {
    volatile motor_mode_t mode;
    volatile float  set_rotor_rps;  /* 速度模式目标 */
    volatile float  set_rotor_deg;  /* 位置模式目标 */
    volatile float  limit_rotor_rps;/* 位置模式限速 */
    volatile int16_t open_out;
    pid_t spd, pos;
} ctrl_t;

static ctrl_t s_ctrl[MAX_MOTORS];

static int valid(int idx)
{
    return idx >= 0 && idx < dji_motor_count() && idx < MAX_MOTORS;
}

void motor_ctrl_init(void)
{
    memset(s_ctrl, 0, sizeof(s_ctrl));
    for (int i = 0; i < dji_motor_count() && i < MAX_MOTORS; i++) {
        const motor_pid_param_t *p = &k_default[dji_motor_cfg(i)->type];
        pid_init(&s_ctrl[i].spd, &p->speed);
        pid_init(&s_ctrl[i].pos, &p->pos);
    }
}

/* 切模式时清积分，防止切换瞬间冲一下（F4 没做） */
static void switch_mode(ctrl_t *c, motor_mode_t m)
{
    if (c->mode != m) {
        pid_reset(&c->spd);
        pid_reset(&c->pos);
        c->mode = m;
    }
}

void motor_ctrl_off(int idx)
{
    if (!valid(idx)) return;
    __disable_irq();
    switch_mode(&s_ctrl[idx], MOTOR_MODE_OFF);
    dji_motor_release(idx);
    __enable_irq();
}

void motor_ctrl_set_open(int idx, int16_t out)
{
    if (!valid(idx)) return;
    __disable_irq();
    s_ctrl[idx].open_out = out;
    switch_mode(&s_ctrl[idx], MOTOR_MODE_OPEN);
    __enable_irq();
}

void motor_ctrl_set_speed(int idx, float shaft_rpm)
{
    if (!valid(idx)) return;
    float g = gear_of(dji_motor_cfg(idx)->type);
    __disable_irq();
    s_ctrl[idx].set_rotor_rps = shaft_rpm * g / 60.0f;  /* 输出轴 rpm -> 转子 rps */
    switch_mode(&s_ctrl[idx], MOTOR_MODE_SPEED);
    __enable_irq();
}

void motor_ctrl_set_pos(int idx, float shaft_deg, float max_shaft_rpm)
{
    if (!valid(idx)) return;
    float g = gear_of(dji_motor_cfg(idx)->type);
    __disable_irq();
    s_ctrl[idx].set_rotor_deg   = shaft_deg * g;        /* 输出轴 ° -> 转子 ° */
    s_ctrl[idx].limit_rotor_rps = (max_shaft_rpm > 0.0f) ? max_shaft_rpm * g / 60.0f : 0.0f;
    switch_mode(&s_ctrl[idx], MOTOR_MODE_POS);
    __enable_irq();
}

void motor_ctrl_set_param(int idx, const motor_pid_param_t *param)
{
    if (!valid(idx) || param == NULL) return;
    __disable_irq();
    pid_init(&s_ctrl[idx].spd, &param->speed);
    pid_init(&s_ctrl[idx].pos, &param->pos);
    __enable_irq();
}

void motor_ctrl_set_gains(int idx, int pos_loop, float kp, float ki, float kd)
{
    if (!valid(idx)) return;
    pid_t *p = pos_loop ? &s_ctrl[idx].pos : &s_ctrl[idx].spd;
    __disable_irq();
    p->p.kp = kp;
    p->p.ki = ki;
    p->p.kd = kd;
    __enable_irq();
}

float motor_ctrl_gear(int idx)
{
    return valid(idx) ? gear_of(dji_motor_cfg(idx)->type) : 1.0f;
}

motor_mode_t motor_ctrl_mode(int idx)
{
    return valid(idx) ? s_ctrl[idx].mode : MOTOR_MODE_OFF;
}

const pid_t *motor_ctrl_speed_pid(int idx) { return valid(idx) ? &s_ctrl[idx].spd : NULL; }
const pid_t *motor_ctrl_pos_pid(int idx)   { return valid(idx) ? &s_ctrl[idx].pos : NULL; }

void motor_ctrl_update(void)
{
    for (int i = 0; i < dji_motor_count() && i < MAX_MOTORS; i++) {
        ctrl_t *c = &s_ctrl[i];
        if (c->mode == MOTOR_MODE_OFF) continue;

        /* 离线：输出 0 并清积分，重新上线时不会突然冲出去 */
        if (!dji_motor_is_online(i)) {
            pid_reset(&c->spd);
            pid_reset(&c->pos);
            dji_motor_set_output(i, 0);
            continue;
        }

        const dji_motor_t *m = dji_motor_get(i);
        float rotor_rps = (float)m->speed_rpm / 60.0f;  /* F4: msg->speed / 60.0f */
        float out = 0.0f;

        switch (c->mode) {
        case MOTOR_MODE_OPEN:
            out = c->open_out;
            break;
        case MOTOR_MODE_SPEED:
            out = pid_calc(&c->spd, c->set_rotor_rps, rotor_rps);
            break;
        case MOTOR_MODE_POS: {
            /* 限速：临时替换位置环输出上限（F4: pid->out_max_out = setspeed） */
            float def = c->pos.p.max_out;
            if (c->limit_rotor_rps > 0.0f) c->pos.p.max_out = c->limit_rotor_rps;
            float spd_set = pid_calc(&c->pos, c->set_rotor_deg, m->angle_deg);
            c->pos.p.max_out = def;
            out = pid_calc(&c->spd, spd_set, rotor_rps);
            break;
        }
        default:
            break;
        }
        dji_motor_set_output(i, (int16_t)out);
    }
    dji_motor_send_all();
}
