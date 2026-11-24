/**
 * @file  dji_motor.c
 * @brief DJI 电机驱动实现，见 dji_motor.h。
 */
#include "dji_motor.h"
#include "motor_cfg.h"
#include <stdio.h>
#include <string.h>

/* ---------------- 型号参数 ---------------- */

typedef struct {
    const char *name;
    uint8_t     id_max;        /* 电调 ID 上限 */
    uint16_t    rx_base;       /* 反馈 ID = rx_base + esc_id */
    int16_t     out_limit;
    float       gear_ratio;
} dji_type_info_t;

static const dji_type_info_t k_type[DJI_TYPE_NUM] = {
    [DJI_M3508]  = { "M3508",  8, 0x200, 16384, 3591.0f / 187.0f },
    [DJI_M2006]  = { "M2006",  8, 0x200, 10000, 36.0f },
    /* 6020 电压模式。老固件手册写 ±30000，新手册是 ±25000，这里按保守值限幅 */
    [DJI_GM6020] = { "GM6020", 7, 0x204, 25000, 1.0f },
};

/* 控制帧：每路总线最多 3 帧 */
enum { CTRL_200 = 0, CTRL_1FF, CTRL_2FF, CTRL_NUM };
static const uint16_t k_ctrl_id[CTRL_NUM] = { 0x200, 0x1FF, 0x2FF };

/* 反馈 ID 0x201~0x20B 共 11 个 */
#define RX_ID_MIN 0x201
#define RX_ID_NUM 11

#define MAX_MOTORS 24
#define MAX_ERRORS 8

typedef struct {
    int8_t  lut[RX_ID_NUM];          /* 反馈 ID -> 电机下标，-1 表示没有 */
    int16_t out[CTRL_NUM][4];
    uint8_t used[CTRL_NUM];          /* 这一帧里有没有已启用的电机，每次 send_all 重新算 */
} bus_ctx_t;

static bus_ctx_t   s_bus[3];
static dji_motor_t s_motor[MAX_MOTORS];
static uint8_t     s_valid[MAX_MOTORS];
static uint8_t     s_ctrl_frame[MAX_MOTORS];
static uint8_t     s_ctrl_slot[MAX_MOTORS];
static int16_t     s_out[MAX_MOTORS];
static uint8_t     s_active[MAX_MOTORS];   /* 调用过 set_output 才为 1，只发启用电机所在的帧 */
static uint8_t     s_release[MAX_MOTORS];  /* release 后还要再发一次 0 才停发 */

static char s_err[MAX_ERRORS][64];
static int  s_err_num;

static void add_error(const char *msg)
{
    if (s_err_num < MAX_ERRORS) {
        strncpy(s_err[s_err_num], msg, sizeof(s_err[0]) - 1);
        s_err[s_err_num][sizeof(s_err[0]) - 1] = '\0';
    }
    s_err_num++;
}

static bus_ctx_t *bus_of(FDCAN_HandleTypeDef *h)
{
    int n = bsp_fdcan_index(h);
    return n ? &s_bus[n - 1] : NULL;
}

/* ---------------- 接收解析（对应 F4 的 Get_msg） ---------------- */

static void parse_feedback(int idx, const uint8_t *d)
{
    dji_motor_t *m = &s_motor[idx];
    float gear = k_type[g_dji_motor_cfg[idx].type].gear_ratio;

    m->ecd         = (uint16_t)(d[0] << 8 | d[1]);
    m->speed_rpm   = (int16_t)(d[2] << 8 | d[3]);
    m->current     = (int16_t)(d[4] << 8 | d[5]);
    m->temperature = d[6];

    if (!m->has_data) {
        /* 第一帧：记零点（F4 里 offset_sign 那段） */
        m->offset_ecd = m->ecd;
        m->last_ecd   = m->ecd;
        m->round_cnt  = 0;
        m->has_data   = 1;
    }

    int32_t diff = (int32_t)m->ecd - (int32_t)m->last_ecd;
    if (diff > 4096)       m->round_cnt--;
    else if (diff < -4096) m->round_cnt++;
    m->last_ecd = m->ecd;

    m->total_ecd     = m->round_cnt * 8192 + (int32_t)m->ecd - (int32_t)m->offset_ecd;
    m->angle_deg     = (float)m->total_ecd * (360.0f / 8192.0f);
    m->out_angle_deg = m->angle_deg / gear;
    m->out_speed_rpm = (float)m->speed_rpm / gear;

    m->last_rx_tick = HAL_GetTick();
    m->rx_cnt++;
}

static uint8_t dji_rx_cb(FDCAN_HandleTypeDef *hfdcan, const can_frame_t *f)
{
    if (f->is_ext || f->len != 8) return 0;
    if (f->id < RX_ID_MIN || f->id >= RX_ID_MIN + RX_ID_NUM) return 0;

    bus_ctx_t *bus = bus_of(hfdcan);
    if (bus == NULL) return 0;

    int idx = bus->lut[f->id - RX_ID_MIN];
    if (idx < 0) return 0;  /* 这个 ID 不在配置表里，交给别的驱动 */

    parse_feedback(idx, f->data);
    return 1;
}

/* ---------------- 初始化：检查配置表 ---------------- */

int dji_motor_init(void)
{
    char msg[64];

    memset(s_bus, 0, sizeof(s_bus));
    memset(s_motor, 0, sizeof(s_motor));
    memset(s_valid, 0, sizeof(s_valid));
    memset(s_out, 0, sizeof(s_out));
    memset(s_active, 0, sizeof(s_active));
    memset(s_release, 0, sizeof(s_release));
    for (int b = 0; b < 3; b++) memset(s_bus[b].lut, -1, sizeof(s_bus[b].lut));
    s_err_num = 0;

    if (g_dji_motor_num > MAX_MOTORS) {
        snprintf(msg, sizeof(msg), "too many motors: %d > %d", g_dji_motor_num, MAX_MOTORS);
        add_error(msg);
    }

    int num = g_dji_motor_num > MAX_MOTORS ? MAX_MOTORS : g_dji_motor_num;
    for (int i = 0; i < num; i++) {
        const dji_motor_cfg_t *c = &g_dji_motor_cfg[i];
        const char *name = c->name ? c->name : "?";
        bus_ctx_t *bus = bus_of(c->hcan);

        if (bus == NULL) {
            snprintf(msg, sizeof(msg), "[%d]%s: bad CAN handle", i, name);
            add_error(msg);
            continue;
        }
        if (c->type >= DJI_TYPE_NUM) {
            snprintf(msg, sizeof(msg), "[%d]%s: bad type", i, name);
            add_error(msg);
            continue;
        }
        const dji_type_info_t *t = &k_type[c->type];
        if (c->esc_id < 1 || c->esc_id > t->id_max) {
            snprintf(msg, sizeof(msg), "[%d]%s: %s id must be 1~%d", i, name, t->name, t->id_max);
            add_error(msg);
            continue;
        }

        uint16_t rx_id = t->rx_base + c->esc_id;
        int8_t *slot = &bus->lut[rx_id - RX_ID_MIN];
        if (*slot >= 0) {
            snprintf(msg, sizeof(msg), "[%d]%s: CAN%d rx 0x%03X conflicts with [%d]",
                     i, name, bsp_fdcan_index(c->hcan), rx_id, *slot);
            add_error(msg);
            continue;
        }
        *slot = (int8_t)i;

        /* 控制帧位置 */
        uint8_t frame;
        if (c->type == DJI_GM6020) frame = (c->esc_id <= 4) ? CTRL_1FF : CTRL_2FF;
        else                       frame = (c->esc_id <= 4) ? CTRL_200 : CTRL_1FF;
        s_ctrl_frame[i] = frame;
        s_ctrl_slot[i]  = (uint8_t)((c->esc_id - 1) % 4);
        s_valid[i] = 1;
    }

    bsp_fdcan_register_rx(&hfdcan1, dji_rx_cb);
    bsp_fdcan_register_rx(&hfdcan2, dji_rx_cb);
    bsp_fdcan_register_rx(&hfdcan3, dji_rx_cb);

    return s_err_num;
}

const char *dji_motor_cfg_error(int n)
{
    if (n < 0 || n >= s_err_num || n >= MAX_ERRORS) return NULL;
    return s_err[n];
}

/* ---------------- 查询 ---------------- */

int dji_motor_count(void)
{
    return g_dji_motor_num > MAX_MOTORS ? MAX_MOTORS : g_dji_motor_num;
}

const dji_motor_cfg_t *dji_motor_cfg(int idx)
{
    return (idx >= 0 && idx < dji_motor_count()) ? &g_dji_motor_cfg[idx] : NULL;
}

const dji_motor_t *dji_motor_get(int idx)
{
    return (idx >= 0 && idx < dji_motor_count()) ? &s_motor[idx] : NULL;
}

uint8_t dji_motor_is_online(int idx)
{
    if (idx < 0 || idx >= dji_motor_count() || !s_valid[idx]) return 0;
    const dji_motor_t *m = &s_motor[idx];
    return m->has_data && (HAL_GetTick() - m->last_rx_tick) < DJI_MOTOR_OFFLINE_MS;
}

int16_t dji_motor_output_limit(dji_type_t type)
{
    return type < DJI_TYPE_NUM ? k_type[type].out_limit : 0;
}

const char *dji_motor_type_name(dji_type_t type)
{
    return type < DJI_TYPE_NUM ? k_type[type].name : "?";
}

/* ---------------- 发送（对应 F4 的 Send_CanX_MsgY） ---------------- */

void dji_motor_set_output(int idx, int16_t out)
{
    if (idx < 0 || idx >= dji_motor_count() || !s_valid[idx]) return;
    int16_t lim = k_type[g_dji_motor_cfg[idx].type].out_limit;
    if (out > lim)  out = lim;
    if (out < -lim) out = -lim;
    s_out[idx] = out;
    s_active[idx]  = 1;
    s_release[idx] = 0;
}

uint8_t dji_motor_is_active(int idx)
{
    return (idx >= 0 && idx < dji_motor_count()) ? s_active[idx] : 0;
}

void dji_motor_release(int idx)
{
    if (idx < 0 || idx >= dji_motor_count() || !s_active[idx]) return;
    s_out[idx]     = 0;
    s_release[idx] = 1;  /* 下一次 send_all 发出 0 之后再停发 */
}

void dji_motor_stop_all(void)
{
    memset(s_out, 0, sizeof(s_out));
}

void dji_motor_reset_angle(int idx)
{
    if (idx < 0 || idx >= dji_motor_count()) return;
    dji_motor_t *m = &s_motor[idx];
    __disable_irq();  /* 防止和接收中断同时改 */
    m->offset_ecd = m->ecd;
    m->last_ecd   = m->ecd;
    m->round_cnt  = 0;
    m->total_ecd  = 0;
    m->angle_deg  = 0.0f;
    m->out_angle_deg = 0.0f;
    __enable_irq();
}

void dji_motor_send_all(void)
{
    /* 先把各电机的输出填到各自总线的帧里 */
    for (int b = 0; b < 3; b++) {
        memset(s_bus[b].out, 0, sizeof(s_bus[b].out));
        memset(s_bus[b].used, 0, sizeof(s_bus[b].used));
    }
    for (int i = 0; i < dji_motor_count(); i++) {
        if (!s_valid[i] || !s_active[i]) continue;  /* 没启用的电机不占帧 */
        bus_ctx_t *bus = bus_of(g_dji_motor_cfg[i].hcan);
        bus->out[s_ctrl_frame[i]][s_ctrl_slot[i]] = s_out[i];
        bus->used[s_ctrl_frame[i]] = 1;
        if (s_release[i]) {  /* 这次把 0 发出去，之后就停发 */
            s_active[i]  = 0;
            s_release[i] = 0;
        }
    }

    static FDCAN_HandleTypeDef *const k_h[3] = { &hfdcan1, &hfdcan2, &hfdcan3 };
    for (int b = 0; b < 3; b++) {
        for (int f = 0; f < CTRL_NUM; f++) {
            if (!s_bus[b].used[f]) continue;  /* 只发有启用电机的帧 */
            uint8_t d[8];
            for (int k = 0; k < 4; k++) {
                d[2 * k]     = (uint8_t)(s_bus[b].out[f][k] >> 8);  /* 高字节在前 */
                d[2 * k + 1] = (uint8_t)(s_bus[b].out[f][k]);
            }
            bsp_fdcan_send_std(k_h[b], k_ctrl_id[f], d, 8);
        }
    }
}
