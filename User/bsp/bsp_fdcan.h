/**
 * @file  bsp_fdcan.h
 * @brief FDCAN 底层：启动、收发、按总线分发接收帧。
 *        只管"帧"，不关心是什么电机。上层（dji_motor / dm_motor / vesc）
 *        通过 bsp_fdcan_register_rx() 挂接收回调。
 */
#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "main.h"
#include <stdint.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/** 一帧接收数据（经典 CAN，最多 8 字节） */
typedef struct {
    uint32_t id;      /**< 标准帧 11 位 / 扩展帧 29 位 */
    uint8_t  is_ext;  /**< 1 = 扩展帧 */
    uint8_t  len;     /**< 0~8 */
    uint8_t  data[8];
} can_frame_t;

/**
 * 接收回调：在中断里调用，返回 1 表示这帧被处理了（后面的回调不再收到）。
 * 回调里只做解析，不要做耗时操作。
 */
typedef uint8_t (*can_rx_cb_t)(FDCAN_HandleTypeDef *hfdcan, const can_frame_t *frame);

/** 每路总线最多挂几个接收回调（DJI、达妙、VESC 各一个就够） */
#define BSP_FDCAN_MAX_CB 4

/** 每路总线的统计信息，调试时看 */
typedef struct {
    uint32_t rx_cnt;       /**< 收到的帧数 */
    uint32_t tx_cnt;       /**< 成功放进发送 FIFO 的帧数 */
    uint32_t tx_fail_cnt;  /**< 发送 FIFO 满（总线上没人应答时会一直涨） */
    uint32_t bus_off_cnt;  /**< 进入 bus-off 的次数（已自动恢复） */
} bsp_fdcan_stat_t;

/** 启动三路 FDCAN（在 MX_FDCANx_Init 之后调用一次） */
void bsp_fdcan_init(void);

/** 给某一路总线挂一个接收回调，成功返回 0 */
int bsp_fdcan_register_rx(FDCAN_HandleTypeDef *hfdcan, can_rx_cb_t cb);

/** 发标准帧（DJI、达妙用），成功返回 0 */
int bsp_fdcan_send_std(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data, uint8_t len);

/** 发扩展帧（VESC 用），成功返回 0 */
int bsp_fdcan_send_ext(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len);

/** 取统计信息，hfdcan 无效时返回 NULL */
const bsp_fdcan_stat_t *bsp_fdcan_get_stat(FDCAN_HandleTypeDef *hfdcan);

/** 总线编号（1/2/3），打印用；无效返回 0 */
int bsp_fdcan_index(FDCAN_HandleTypeDef *hfdcan);

#endif
