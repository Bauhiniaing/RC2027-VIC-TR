/**
 * @file  bsp_imu.c
 * @brief HiPNUC 陀螺仪：串口字节 -> 官方解码器 -> 最新数据。说明见 bsp_imu.h
 */
#include "bsp_imu.h"
#include "hipnuc_decoder.h"
#include "imu_uart.h"
#include "main.h"

#define IMU_TIMEOUT_MS 200

static hipnuc_decoder_t s_dec;   /* 全零即可用 */
static hipnuc_data_t s_imu;
static imu_stat_t s_stat;
static uint32_t s_last_frame_ms;

/* HAL 对所有串口都会调这个回调。UART7（ELRS）不走 HAL 中断，不会进来。
 * 以后别的串口也要用这个回调的话，在这里加，不要再定义一次。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    imu_uart_rx_complete(huart);
}

void imu_init(void)
{
    imu_uart_start();
}

void imu_poll(void)
{
    uint8_t buf[64];
    size_t n;

    while ((n = imu_uart_read(buf, sizeof(buf))) > 0) {
        const uint8_t *p = buf;
        int ev;
        while ((ev = hipnuc_decoder_feed(&s_dec, &p, &n, &s_imu)) != 0) {
            if (ev != HIPNUC_EV_LINE) s_last_frame_ms = HAL_GetTick();
            /* ev == HIPNUC_EV_LINE 是模块回的文本行（比如命令的 OK），这里不处理 */
        }
    }

    s_stat.frames   = s_dec.frames;
    s_stat.crc_err  = s_dec.crc_errors;
    s_stat.bytes    = s_dec.bytes;
    s_stat.uart_err = imu_uart_errors;
    s_stat.dropped  = imu_uart_dropped;
}

const hipnuc_data_t *imu_get(void) { return &s_imu; }
const imu_stat_t *imu_get_stat(void) { return &s_stat; }

int imu_is_online(void)
{
    return s_dec.frames > 0 && (HAL_GetTick() - s_last_frame_ms) < IMU_TIMEOUT_MS;
}

float imu_roll_deg(void)  { return s_imu.roll  * HIPNUC_RAD2DEG; }
float imu_pitch_deg(void) { return s_imu.pitch * HIPNUC_RAD2DEG; }
float imu_yaw_deg(void)   { return s_imu.yaw   * HIPNUC_RAD2DEG; }
