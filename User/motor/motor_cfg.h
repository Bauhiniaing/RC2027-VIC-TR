/**
 * @file  motor_cfg.h
 * @brief 电机配置表。加/减电机、换总线、改 ID 只改这里和 motor_cfg.c。
 */
#ifndef MOTOR_CFG_H
#define MOTOR_CFG_H

#include "dji_motor.h"

/* 电机下标，和 motor_cfg.c 表里的顺序一一对应，业务代码里用名字而不是数字 */
enum {
    MOTOR_M3508_1 = 0,  /* FDCAN1 */
    MOTOR_M3508_2,
    MOTOR_M3508_3,
    MOTOR_M3508_4,
    MOTOR_M3508_5,
    MOTOR_M3508_6,
    MOTOR_M3508_7,
    MOTOR_M3508_8,
    MOTOR_M2006_1,      /* FDCAN2 */
    MOTOR_M2006_2,
    MOTOR_M2006_3,
    MOTOR_M2006_4,
    MOTOR_M2006_5,
    MOTOR_M2006_6,
    MOTOR_M2006_7,
    MOTOR_M2006_8,
    MOTOR_GM6020_5,     /* FDCAN2，6020 只能用 ID 5~7（方案 C） */
    MOTOR_GM6020_6,
    MOTOR_GM6020_7,
};

extern const dji_motor_cfg_t g_dji_motor_cfg[];
extern const int             g_dji_motor_num;

#endif
