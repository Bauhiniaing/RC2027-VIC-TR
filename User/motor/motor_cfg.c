/**
 * @file  motor_cfg.c
 * @brief 电机配置表。
 *
 * 分配方案：
 *   FDCAN1 (PD0/PD1)   : 3508 ID 1~8
 *   FDCAN2 (PB5/PB6)   : 2006 ID 1~8 + 6020 ID 5~7
 *   FDCAN3 (PD12/PD13) : 预留给达妙（VESC 待定）
 *
 * 规则：
 *  - 同一路总线上反馈 ID 不能重复，dji_motor_init() 会检查并报错。
 *  - 表里的电机没接也没关系，只会显示离线。
 *  - 6020 在 FDCAN2 上只能用 ID 5~7（反馈 0x209~0x20B、控制帧 0x2FF），和 2006 不冲突。
 *    ⚠ 6020 电调要拨码到 5~7！拨成 1~4 会和 2006 ID5~8 冲突（反馈 0x205~0x208、控制帧 0x1FF）。
 *    要接第 4 个 6020：把 2006 改成只配 1~4，6020 改用 ID 1~4（方案 A）。
 *  - 带宽：一路 8 个电机全部接上时，光反馈就会占满 1Mbps 总线，实际接线每路建议 ≤6 个。
 */
#include "motor_cfg.h"

const dji_motor_cfg_t g_dji_motor_cfg[] = {
    /* 下标               名字        总线       型号       电调ID */
    [MOTOR_M3508_1] = { "m3508_1", &hfdcan1, DJI_M3508, 1 },
    [MOTOR_M3508_2] = { "m3508_2", &hfdcan1, DJI_M3508, 2 },
    [MOTOR_M3508_3] = { "m3508_3", &hfdcan1, DJI_M3508, 3 },
    [MOTOR_M3508_4] = { "m3508_4", &hfdcan1, DJI_M3508, 4 },
    [MOTOR_M3508_5] = { "m3508_5", &hfdcan1, DJI_M3508, 5 },
    [MOTOR_M3508_6] = { "m3508_6", &hfdcan1, DJI_M3508, 6 },
    [MOTOR_M3508_7] = { "m3508_7", &hfdcan1, DJI_M3508, 7 },
    [MOTOR_M3508_8] = { "m3508_8", &hfdcan1, DJI_M3508, 8 },
    [MOTOR_M2006_1] = { "m2006_1", &hfdcan2, DJI_M2006, 1 },
    [MOTOR_M2006_2] = { "m2006_2", &hfdcan2, DJI_M2006, 2 },
    [MOTOR_M2006_3] = { "m2006_3", &hfdcan2, DJI_M2006, 3 },
    [MOTOR_M2006_4] = { "m2006_4", &hfdcan2, DJI_M2006, 4 },
    [MOTOR_M2006_5] = { "m2006_5", &hfdcan2, DJI_M2006, 5 },
    [MOTOR_M2006_6] = { "m2006_6", &hfdcan2, DJI_M2006, 6 },
    [MOTOR_M2006_7] = { "m2006_7", &hfdcan2, DJI_M2006, 7 },
    [MOTOR_M2006_8] = { "m2006_8", &hfdcan2, DJI_M2006, 8 },
    [MOTOR_GM6020_5] = { "gm6020_5", &hfdcan2, DJI_GM6020, 5 },
    [MOTOR_GM6020_6] = { "gm6020_6", &hfdcan2, DJI_GM6020, 6 },
    [MOTOR_GM6020_7] = { "gm6020_7", &hfdcan2, DJI_GM6020, 7 },
};

const int g_dji_motor_num = sizeof(g_dji_motor_cfg) / sizeof(g_dji_motor_cfg[0]);
