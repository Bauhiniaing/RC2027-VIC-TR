/**
 * @file  bsp_crsf.c
 * @brief ELRS 接收机 CRSF 协议：UART7 收通道、发遥测。说明见 bsp_crsf.h
 *
 * CRSF 帧格式：[sync][len][type][payload...][crc]
 *   len = type + payload + crc 的字节数；crc 为 CRC8(多项式 0xD5)，从 type 算到 payload 末尾。
 */
#include "bsp_crsf.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart7;   /* CubeMX 生成在 main.c：420000 8N1，FIFO 关，PE7/PE8 */
#define CRSF_UART            UART7

#define CRSF_SYNC_FC         0xC8       /* 接收机 → 飞控 用的地址，回传也用它 */
#define CRSF_SYNC_RX         0xEC
#define CRSF_SYNC_TX_MODULE  0xEE
#define CRSF_MAX_FRAME       64

#define CRSF_TYPE_BATTERY    0x08
#define CRSF_TYPE_LINK       0x14
#define CRSF_TYPE_CHANNELS   0x16
#define CRSF_TYPE_ATTITUDE   0x1E
#define CRSF_TYPE_FLIGHTMODE 0x21

static volatile uint16_t s_ch[CRSF_CH_NUM] = {
    CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID,
    CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID, CRSF_CH_MID,
};
static volatile uint32_t s_last_ch_ms;
static volatile uint8_t  s_got_ch;      /* 上电后收到过至少一帧通道 */
static volatile crsf_link_t s_link;
static volatile crsf_stat_t s_stat;

/* ------------------------------------------------------------------ CRC */
static uint8_t crc8_d5(const uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ------------------------------------------------------------------ 解析（中断里调用） */
static void unpack_channels(const uint8_t *p)   /* 22 字节 = 16 × 11bit，低位在前 */
{
    uint32_t bits = 0;
    uint8_t  n = 0, ch = 0;
    for (int i = 0; i < 22 && ch < CRSF_CH_NUM; i++) {
        bits |= (uint32_t)p[i] << n;
        n += 8;
        while (n >= 11 && ch < CRSF_CH_NUM) {
            s_ch[ch++] = bits & 0x7FF;
            bits >>= 11;
            n -= 11;
        }
    }
}

static void handle_frame(const uint8_t *f)
{
    uint8_t type = f[2];
    const uint8_t *pl = &f[3];
    uint8_t pl_len = f[1] - 2;

    switch (type) {
    case CRSF_TYPE_CHANNELS:
        if (pl_len < 22) return;
        unpack_channels(pl);
        s_last_ch_ms = HAL_GetTick();
        s_got_ch = 1;
        break;
    case CRSF_TYPE_LINK:
        if (pl_len < 10) return;
        s_link.up_rssi1    = pl[0];
        s_link.up_rssi2    = pl[1];
        s_link.up_lq       = pl[2];
        s_link.up_snr      = (int8_t)pl[3];
        s_link.antenna     = pl[4];
        s_link.rf_mode     = pl[5];
        s_link.up_tx_power = pl[6];
        s_link.down_rssi   = pl[7];
        s_link.down_lq     = pl[8];
        s_link.down_snr    = (int8_t)pl[9];
        break;
    default:
        break;   /* 其他帧（设备信息、参数等）不处理 */
    }
}

static void rx_byte(uint8_t c)
{
    static uint8_t buf[CRSF_MAX_FRAME];
    static uint8_t pos;

    if (pos == 0) {
        if (c == CRSF_SYNC_FC || c == CRSF_SYNC_RX || c == CRSF_SYNC_TX_MODULE) buf[pos++] = c;
        return;
    }
    if (pos == 1) {
        if (c < 2 || c > CRSF_MAX_FRAME - 2) {   /* 长度不合理：这不是帧头，重新找 */
            pos = 0;
            return;
        }
        buf[pos++] = c;
        return;
    }
    buf[pos++] = c;
    if (pos == buf[1] + 2) {
        if (crc8_d5(&buf[2], buf[1] - 1) == buf[pos - 1]) {
            s_stat.frames_ok++;
            handle_frame(buf);
        } else {
            s_stat.crc_err++;
        }
        pos = 0;
    }
}

/* 在 stm32h7xx_it.c 的 UART7_IRQHandler 里调用（USER CODE UART7_IRQn 0） */
void crsf_uart_irq(void)
{
    uint32_t isr = CRSF_UART->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        CRSF_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        s_stat.uart_err++;
    }
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)CRSF_UART->RDR;
        s_stat.rx_bytes++;
        rx_byte(c);
    }
}

/* ------------------------------------------------------------------ 初始化 */
void crsf_init(void)
{
    /* 串口本身由 CubeMX 的 MX_UART7_Init() 配好，这里只打开接收中断和错误中断 */
    CRSF_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_ERR);
}

/* ------------------------------------------------------------------ 读取接口 */
int crsf_is_connected(void)
{
    return s_got_ch && (HAL_GetTick() - s_last_ch_ms < CRSF_TIMEOUT_MS);
}

uint16_t crsf_get_raw(int ch)
{
    if (ch < 0 || ch >= CRSF_CH_NUM) return CRSF_CH_MID;
    return s_ch[ch];
}

uint16_t crsf_get_us(int ch)
{
    /* 172→988，992→1500，1811→2012；和 ELRS/Betaflight 的换算一致 */
    return (uint16_t)((crsf_get_raw(ch) - CRSF_CH_MID) * 5 / 8 + 1500);
}

float crsf_get_axis(int ch)
{
    if (!crsf_is_connected()) return 0.0f;
    float v = ((float)crsf_get_raw(ch) - CRSF_CH_MID) / (float)(CRSF_CH_MAX - CRSF_CH_MID);
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > -0.02f && v < 0.02f) v = 0.0f;     /* 中位小死区，防止摇杆回中时轻微漂移 */
    return v;
}

int crsf_get_switch(int ch)
{
    if (!crsf_is_connected()) return 0;
    uint16_t us = crsf_get_us(ch);
    if (us < 1300) return 0;
    if (us > 1700) return 2;
    return 1;
}

rc_sticks_t crsf_get_sticks(void)
{
    rc_sticks_t s;
    s.lx = RC_LX_SIGN * crsf_get_axis(RC_LX_CH);
    s.ly = RC_LY_SIGN * crsf_get_axis(RC_LY_CH);
    s.rx = RC_RX_SIGN * crsf_get_axis(RC_RX_CH);
    s.ry = RC_RY_SIGN * crsf_get_axis(RC_RY_CH);
    return s;
}

int crsf_get_button(int n)
{
    if (n < 1 || n > RC_BTN_NUM || !crsf_is_connected()) return 0;
    return crsf_get_us(RC_BTN_FIRST_CH + n - 1) > 1500;
}

const crsf_link_t *crsf_get_link(void) { return (const crsf_link_t *)&s_link; }
const crsf_stat_t *crsf_get_stat(void) { return (const crsf_stat_t *)&s_stat; }

/* ------------------------------------------------------------------ 发送遥测 */
static void uart_send(const uint8_t *p, uint8_t len)
{
    HAL_UART_Transmit(&huart7, (uint8_t *)p, len, 5);   /* 阻塞发送，最多等 5ms */
}

/* 组帧并发送：payload 放在 f[3] 开始，函数填头和 CRC */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t pl_len)
{
    uint8_t f[CRSF_MAX_FRAME];
    if (pl_len > CRSF_MAX_FRAME - 4) return;
    f[0] = CRSF_SYNC_FC;
    f[1] = pl_len + 2;
    f[2] = type;
    memcpy(&f[3], payload, pl_len);
    f[3 + pl_len] = crc8_d5(&f[2], pl_len + 1);
    uart_send(f, pl_len + 4);
}

static void put_be16(uint8_t *p, int32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

void crsf_send_battery(float voltage, float current, uint32_t used_mah, uint8_t remaining)
{
    uint8_t p[8];
    put_be16(&p[0], (int32_t)(voltage * 10.0f + 0.5f));   /* 0.1V */
    put_be16(&p[2], (int32_t)(current * 10.0f + 0.5f));   /* 0.1A */
    p[4] = (uint8_t)(used_mah >> 16);
    p[5] = (uint8_t)(used_mah >> 8);
    p[6] = (uint8_t)used_mah;
    p[7] = remaining;
    send_frame(CRSF_TYPE_BATTERY, p, sizeof(p));
}

void crsf_send_attitude(float pitch, float roll, float yaw)
{
    uint8_t p[6];
    put_be16(&p[0], (int32_t)(pitch * 10000.0f));   /* 单位 100µrad */
    put_be16(&p[2], (int32_t)(roll  * 10000.0f));
    put_be16(&p[4], (int32_t)(yaw   * 10000.0f));
    send_frame(CRSF_TYPE_ATTITUDE, p, sizeof(p));
}

void crsf_send_flight_mode(const char *mode)
{
    uint8_t p[16];
    uint8_t n = 0;
    while (mode[n] && n < 15) { p[n] = (uint8_t)mode[n]; n++; }
    p[n++] = 0;   /* 以 '\0' 结尾 */
    send_frame(CRSF_TYPE_FLIGHTMODE, p, n);
}
