/**
 * @file  bsp_imu.h
 * @brief HiPNUC HI14 陀螺仪（RS-232 版，经 MAX3232 转 TTL）串口接收。
 *
 * 用法：
 *   app_init() 里调 imu_init();
 *   app_loop() 里一直调 imu_poll();   （把串口收到的字节喂给官方解码器）
 *   任何地方用 imu_get() 读最新数据，先看 valid 位。
 * 串口在 imu_uart.h 的 IMU_UART_HANDLE 里选，默认 115200 8N1，中断接收。
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include <stdint.h>
#include "hipnuc_data.h"

typedef struct {
    uint32_t frames;      /* 解出来的帧数 */
    uint32_t crc_err;     /* CRC 错误帧 */
    uint32_t uart_err;    /* 串口溢出/重启次数 */
    uint32_t dropped;     /* 缓冲满丢掉的字节 */
    uint32_t bytes;       /* 收到的总字节 */
} imu_stat_t;

void imu_init(void);
void imu_poll(void);                    /* 主循环里调，不要放进中断 */
const hipnuc_data_t *imu_get(void);     /* 最新一帧数据 */
const imu_stat_t *imu_get_stat(void);
int imu_is_online(void);                /* 200 ms 内收到过帧 */

/* 常用量，单位：角度 deg，角速度 deg/s */
float imu_roll_deg(void);
float imu_pitch_deg(void);
float imu_yaw_deg(void);

#endif
