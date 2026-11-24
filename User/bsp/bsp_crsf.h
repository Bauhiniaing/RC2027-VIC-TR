/**
 * @file  bsp_crsf.h
 * @brief ELRS 接收机 CRSF 串口驱动（UART7：PE7 RX / PE8 TX，420000 8N1 不反相）
 *
 * 接线：接收机 TX → PE7(UART7_RX)，接收机 RX → PE8(UART7_TX)，5V、GND。
 * 收：中断里逐字节解析，CRC 校验通过的 0x16 通道帧 / 0x14 链路统计帧更新到内部数据。
 * 发：crsf_send_battery() 等函数把遥测帧发给接收机，ELRS 再无线回传给遥控器。
 *
 * 串口由 CubeMX 配置（UART7，420000，FIFO 关，NVIC 开）。中断入口在 stm32h7xx_it.c：
 *   USER CODE BEGIN UART7_IRQn 0 里调用 crsf_uart_irq(); return;（不走 HAL 的接收流程）
 */
#ifndef BSP_CRSF_H
#define BSP_CRSF_H

#include <stdint.h>

#define CRSF_CH_NUM        16
#define CRSF_TIMEOUT_MS    200   /* 超过这么久没收到通道帧就算失控 */

/* 通道原始值范围（ELRS 默认）：172 = 最小，992 = 中位，1811 = 最大 */
#define CRSF_CH_MIN        172
#define CRSF_CH_MID        992
#define CRSF_CH_MAX        1811

typedef struct {
    uint8_t up_rssi1;      /* 上行 RSSI（接收机收到的信号），单位 -dBm */
    uint8_t up_rssi2;
    uint8_t up_lq;         /* 上行链路质量 0~100% */
    int8_t  up_snr;
    uint8_t antenna;
    uint8_t rf_mode;
    uint8_t up_tx_power;
    uint8_t down_rssi;     /* 下行（回传）RSSI，-dBm */
    uint8_t down_lq;
    int8_t  down_snr;
} crsf_link_t;

typedef struct {
    uint32_t rx_bytes;     /* 收到的字节数：一直是 0 → 接线/波特率不对 */
    uint32_t frames_ok;    /* CRC 正确的帧数 */
    uint32_t crc_err;      /* CRC 错误：偶尔有正常，一直涨 → 线太长/干扰/波特率 */
    uint32_t uart_err;     /* 溢出、帧错误、噪声 */
} crsf_stat_t;

void crsf_init(void);      /* 在 MX_UART7_Init() 之后调用 */
void crsf_uart_irq(void); /* UART7 中断里调用 */

/* 1 = 正在收到通道帧；0 = 失控（接收机没对上频 / 遥控器关了 / 线断了） */
int crsf_is_connected(void);

/* 原始值 172~1811，失控时返回最后一次的值 */
uint16_t crsf_get_raw(int ch);

/* 映射到 1000~2000us（和遥控器那边 CH_out 的单位一样） */
uint16_t crsf_get_us(int ch);

/* 映射到 -1.0 ~ +1.0，中位 0。适合摇杆通道，失控时返回 0 */
float crsf_get_axis(int ch);

/* 拨杆三档：返回 0 / 1 / 2（低/中/高），失控时返回 0 */
int crsf_get_switch(int ch);

/* ---- 摇杆（已按实测方向统一：往前推 = 正，往右推 = 正，范围 -1~+1） ----
 * 实测（2026-11-21，遥控器当前 output_map）：
 *   左摇杆 前推 → CH4(下标3) 增大    左摇杆 左推 → CH3(下标2) 增大
 *   右摇杆 前推 → CH1(下标0) 减小    右摇杆 右推 → CH2(下标1) 减小
 * 遥控器那边改了通道映射或反向，只改下面这几个宏。 */
#define RC_LY_CH    3
#define RC_LY_SIGN  (+1)
#define RC_LX_CH    2
#define RC_LX_SIGN  (-1)
#define RC_RY_CH    0
#define RC_RY_SIGN  (-1)
#define RC_RX_CH    1
#define RC_RX_SIGN  (-1)

typedef struct {
    float lx, ly;   /* 左摇杆：右 +，前 + */
    float rx, ry;   /* 右摇杆：右 +，前 + */
} rc_sticks_t;

/* 失控时四个值全是 0 */
rc_sticks_t crsf_get_sticks(void);

const crsf_link_t *crsf_get_link(void);
const crsf_stat_t *crsf_get_stat(void);

/* ---- 回传（只在主循环里调用，阻塞发送，十几个字节不到 0.5ms） ----
 * 建议 1~5Hz 发一次，ELRS 回传带宽很小，发多了会被丢。
 * 遥控器那边的 Telemetry Ratio 不能是 Off。 */

/* 电池：电压 V、电流 A、已用容量 mAh、剩余百分比 */
void crsf_send_battery(float voltage, float current, uint32_t used_mah, uint8_t remaining);

/* 姿态：单位弧度 */
void crsf_send_attitude(float pitch, float roll, float yaw);

/* 飞行模式字符串，例如 "AUTO"，最多 15 个字符，遥控器屏幕上可以显示当前状态 */
void crsf_send_flight_mode(const char *mode);

#endif
