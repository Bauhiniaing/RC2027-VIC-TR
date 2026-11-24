/**
 * @file  dji_motor.h
 * @brief DJI M3508(C620) / M2006(C610) / GM6020 驱动。
 *        从 F4 工程 Motor_can.c 移植，改成配置表驱动：电机挂在哪路 CAN、
 *        ID 是几，都写在 motor_cfg.c 的表里，这里自动解析反馈、自动打包控制帧。
 *
 * 协议速查（以 DJI 官方手册为准）：
 *  | 型号        | 电调ID | 反馈ID         | 控制帧  | 输出范围         |
 *  |-------------|--------|----------------|---------|------------------|
 *  | 3508 / 2006 | 1~4    | 0x201~0x204    | 0x200   | 3508 ±16384 电流 |
 *  | 3508 / 2006 | 5~8    | 0x205~0x208    | 0x1FF   | 2006 ±10000 电流 |
 *  | 6020        | 1~4    | 0x205~0x208    | 0x1FF   | ±25000 电压      |
 *  | 6020        | 5~7    | 0x209~0x20B    | 0x2FF   |                  |
 *  同一路总线上反馈 ID 不能重复，dji_motor_init() 会检查。
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_fdcan.h"
#include <stdint.h>

typedef enum {
    DJI_M3508 = 0,
    DJI_M2006,
    DJI_GM6020,
    DJI_TYPE_NUM
} dji_type_t;

/** 配置表的一行 */
typedef struct {
    const char          *name;   /**< 名字，打印用，例如 "chassis_lf" */
    FDCAN_HandleTypeDef *hcan;   /**< &hfdcan1 / &hfdcan2 / &hfdcan3 */
    dji_type_t           type;
    uint8_t              esc_id; /**< 电调上设置的 ID：3508/2006 为 1~8，6020 为 1~7 */
} dji_motor_cfg_t;

/** 电机反馈（对应 F4 的 Motor_Msg） */
typedef struct {
    /* 电调原始数据 */
    uint16_t ecd;          /**< 转子机械角度 0~8191（F4: angle） */
    int16_t  speed_rpm;    /**< 转子转速 rpm（F4: speed） */
    int16_t  current;      /**< 实际转矩电流（F4: torque） */
    uint8_t  temperature;  /**< 温度 ℃（F4: temperate） */

    /* 多圈角度（上电第一帧为零点，和 F4 get_moto_offset 一样） */
    uint16_t offset_ecd;   /**< 零点（F4: offset_angle） */
    uint16_t last_ecd;     /**< 上一帧角度（F4: last_angle） */
    int32_t  round_cnt;    /**< 圈数（F4: round_cnt） */
    int32_t  total_ecd;    /**< 累计编码值（F4: total_angle） */
    float    angle_deg;    /**< 转子累计角度 °（F4: real_angle） */

    /* 输出轴（已除减速比：3508 为 3591/187，2006 为 36，6020 为 1） */
    float    out_angle_deg;
    float    out_speed_rpm;

    /* 状态 */
    uint32_t last_rx_tick; /**< 最近一次收到反馈的 HAL_GetTick() */
    uint32_t rx_cnt;
    uint8_t  has_data;     /**< 收到过至少一帧 */
} dji_motor_t;

/** 判定离线的超时时间（ms），DJI 电调正常是 1kHz 回传 */
#define DJI_MOTOR_OFFLINE_MS 100

/**
 * 检查配置表并注册 CAN 回调，在 bsp_fdcan_init() 之后调用。
 * @return 配置错误的条数（错误的电机会被忽略），0 表示全部正常。
 *         错误原因用 dji_motor_cfg_error() 获取。
 */
int dji_motor_init(void);

/** 第 n 条配置错误的说明，没有则返回 NULL */
const char *dji_motor_cfg_error(int n);

/** 配置表里的电机数量 */
int dji_motor_count(void);

const dji_motor_cfg_t *dji_motor_cfg(int idx);
const dji_motor_t     *dji_motor_get(int idx);

/** 是否在线（DJI_MOTOR_OFFLINE_MS 内收到过反馈） */
uint8_t dji_motor_is_online(int idx);

/**
 * 设置输出（3508/2006 为电流，6020 为电压），超出范围自动限幅。
 * 第一次调用时启用这个电机，之后 send_all 会一直发它所在的控制帧（不调用的电机不发）。
 */
void dji_motor_set_output(int idx, int16_t out);

/** 停用电机：下一次 send_all 先发一帧 0，然后不再发它所在的帧（帧里没有其他启用的电机时） */
void dji_motor_release(int idx);

/** 是否已启用 */
uint8_t dji_motor_is_active(int idx);

/** 已启用电机的输出全部清零，仍然继续发 0（急停用，记得再调一次 send_all） */
void dji_motor_stop_all(void);

/** 把已启用电机的输出打包发出去，按固定周期调用（建议 1kHz） */
void dji_motor_send_all(void);

/** 把当前位置设为零点（F4: get_moto_offset） */
void dji_motor_reset_angle(int idx);

/** 型号的输出上限 */
int16_t dji_motor_output_limit(dji_type_t type);

const char *dji_motor_type_name(dji_type_t type);

#endif
