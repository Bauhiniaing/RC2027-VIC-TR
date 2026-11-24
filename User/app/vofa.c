#include "vofa.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "dji_motor.h"
#include "motor_ctrl.h"
#include <stdlib.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define CH_NUM     6
#define FRAME_NUM  128  /* 环形缓冲能存 128ms 的波形 */

typedef struct {
    float   ch[CH_NUM];
    uint8_t tail[4];    /* JustFloat 帧尾 00 00 80 7F */
} frame_t;

static frame_t           s_frame[FRAME_NUM];
static volatile uint16_t s_head, s_tail;
static volatile int      s_motor;
static volatile uint16_t s_div = 1, s_div_cnt;
static float             s_pos_lim;

static uint8_t           s_tx[16 * sizeof(frame_t)];

static char              s_rx[256];
static volatile uint16_t s_rx_head, s_rx_tail;

void vofa_init(int motor_idx)
{
    s_motor = motor_idx;
}

/* ---------------- 采样（中断里） ---------------- */

void vofa_sample_isr(void)
{
    if (++s_div_cnt < s_div) return;
    s_div_cnt = 0;

    uint16_t next = (s_head + 1) % FRAME_NUM;
    if (next == s_tail) return;  /* 满了（USB 没在读），丢掉这一帧 */

    int i = s_motor;
    const dji_motor_t *m = dji_motor_get(i);
    const pid_t *ps = motor_ctrl_speed_pid(i);
    const pid_t *pp = motor_ctrl_pos_pid(i);
    if (!m || !ps || !pp) return;

    float g = motor_ctrl_gear(i);
    frame_t *f = &s_frame[s_head];
    motor_mode_t mode = motor_ctrl_mode(i);
    f->ch[0] = (mode == MOTOR_MODE_POS) ? pp->set / g : m->out_angle_deg;
    f->ch[1] = m->out_angle_deg;
    f->ch[2] = (mode == MOTOR_MODE_POS || mode == MOTOR_MODE_SPEED) ? ps->set * 60.0f / g : 0.0f;
    f->ch[3] = m->out_speed_rpm;
    f->ch[4] = (mode == MOTOR_MODE_POS || mode == MOTOR_MODE_SPEED) ? ps->out : 0.0f;
    f->ch[5] = m->current;
    f->tail[0] = 0x00; f->tail[1] = 0x00; f->tail[2] = 0x80; f->tail[3] = 0x7F;
    s_head = next;
}

/* ---------------- 接收（USB 中断里） ---------------- */

void vofa_on_usb_rx(const uint8_t *buf, uint32_t len)
{
    for (uint32_t k = 0; k < len; k++) {
        uint16_t next = (s_rx_head + 1) % sizeof(s_rx);
        if (next == s_rx_tail) break;
        s_rx[s_rx_head] = (char)buf[k];
        s_rx_head = next;
    }
}

/* ---------------- 命令解析（主循环） ---------------- */

static void exec(char *line)
{
    char *eq = strchr(line, '=');
    float v = eq ? strtof(eq + 1, NULL) : 0.0f;
    if (eq) *eq = '\0';
    int i = s_motor;
    const pid_t *ps = motor_ctrl_speed_pid(i);
    const pid_t *pp = motor_ctrl_pos_pid(i);
    if (!ps || !pp) return;

    if      (!strcmp(line, "m"))   { if ((int)v >= 0 && (int)v < dji_motor_count()) s_motor = (int)v; }
    else if (!strcmp(line, "pos")) motor_ctrl_set_pos(i, v, s_pos_lim);
    else if (!strcmp(line, "spd")) motor_ctrl_set_speed(i, v);
    else if (!strcmp(line, "off")) motor_ctrl_off(i);
    else if (!strcmp(line, "lim")) s_pos_lim = v;
    else if (!strcmp(line, "div")) s_div = (v >= 1.0f) ? (uint16_t)v : 1;
    else if (!strcmp(line, "skp")) motor_ctrl_set_gains(i, 0, v, ps->p.ki, ps->p.kd);
    else if (!strcmp(line, "ski")) motor_ctrl_set_gains(i, 0, ps->p.kp, v, ps->p.kd);
    else if (!strcmp(line, "skd")) motor_ctrl_set_gains(i, 0, ps->p.kp, ps->p.ki, v);
    else if (!strcmp(line, "pkp")) motor_ctrl_set_gains(i, 1, v, pp->p.ki, pp->p.kd);
    else if (!strcmp(line, "pki")) motor_ctrl_set_gains(i, 1, pp->p.kp, v, pp->p.kd);
    else if (!strcmp(line, "pkd")) motor_ctrl_set_gains(i, 1, pp->p.kp, pp->p.ki, v);
}

static void poll_rx(void)
{
    static char line[32];
    static uint8_t n;
    while (s_rx_tail != s_rx_head) {
        char c = s_rx[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % sizeof(s_rx);
        if (c == '\n' || c == '\r' || c == ';') {
            if (n) { line[n] = '\0'; exec(line); n = 0; }
        } else if (c != ' ' && n < sizeof(line) - 1) {
            line[n++] = c;
        }
    }
}

/* ---------------- 发送（主循环） ---------------- */

static void poll_tx(void)
{
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceHS.pClassData == NULL) {
        s_tail = s_head;  /* 没连上：丢掉 */
        return;
    }
    if (((USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData)->TxState != 0) return;  /* 上一包还在发 */

    uint16_t n = 0;
    while (s_tail != s_head && n < sizeof(s_tx) / sizeof(frame_t)) {
        memcpy(&s_tx[n * sizeof(frame_t)], &s_frame[s_tail], sizeof(frame_t));
        s_tail = (s_tail + 1) % FRAME_NUM;
        n++;
    }
    if (n) CDC_Transmit_HS(s_tx, n * sizeof(frame_t));
}

void vofa_poll(void)
{
    poll_rx();
    poll_tx();
}
