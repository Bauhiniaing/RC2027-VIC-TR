/**
 * @file  bsp_fdcan.c
 * @brief FDCAN 底层实现。
 *
 * 和 F4 bxCAN 的主要区别：
 *  - F4: HAL_CAN_ConfigFilter / HAL_CAN_Start / HAL_CAN_AddTxMessage / HAL_CAN_GetRxMessage
 *  - H7: HAL_FDCAN_ConfigGlobalFilter / HAL_FDCAN_Start / HAL_FDCAN_AddMessageToTxFifoQ / HAL_FDCAN_GetRxMessage
 *  - H7 的 DataLength 是 FDCAN_DLC_BYTES_x 宏（经典 CAN 下 0~8 和宏的值相同，但仍按宏写）
 *  - H7 bus-off 后不会自动恢复，需要软件清 INIT 位（见 HAL_FDCAN_ErrorStatusCallback）
 */
#include "bsp_fdcan.h"
#include <string.h>

typedef struct {
    FDCAN_HandleTypeDef *h;
    can_rx_cb_t          cb[BSP_FDCAN_MAX_CB];
    uint8_t              cb_num;
    bsp_fdcan_stat_t     stat;
} fdcan_bus_t;

static fdcan_bus_t s_bus[3] = {
    { .h = &hfdcan1 },
    { .h = &hfdcan2 },
    { .h = &hfdcan3 },
};

static fdcan_bus_t *find_bus(FDCAN_HandleTypeDef *hfdcan)
{
    for (int i = 0; i < 3; i++) {
        if (s_bus[i].h == hfdcan) return &s_bus[i];
    }
    return NULL;
}

int bsp_fdcan_index(FDCAN_HandleTypeDef *hfdcan)
{
    for (int i = 0; i < 3; i++) {
        if (s_bus[i].h == hfdcan) return i + 1;
    }
    return 0;
}

static void start_one(FDCAN_HandleTypeDef *h)
{
    /* 不配单独的过滤器：标准帧、扩展帧全部收进 FIFO0，远程帧丢掉。
     * 一路上最多也就十几个节点，在软件里按 ID 分发足够快。 */
    if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(h) != HAL_OK) {
        Error_Handler();
    }
}

void bsp_fdcan_init(void)
{
    for (int i = 0; i < 3; i++) {
        start_one(s_bus[i].h);
    }
}

int bsp_fdcan_register_rx(FDCAN_HandleTypeDef *hfdcan, can_rx_cb_t cb)
{
    fdcan_bus_t *bus = find_bus(hfdcan);
    if (bus == NULL || cb == NULL || bus->cb_num >= BSP_FDCAN_MAX_CB) return -1;
    for (int i = 0; i < bus->cb_num; i++) {
        if (bus->cb[i] == cb) return 0; /* 重复注册直接忽略 */
    }
    bus->cb[bus->cb_num++] = cb;
    return 0;
}

static const uint32_t k_dlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

static int send_frame(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint32_t id_type,
                      const uint8_t *data, uint8_t len)
{
    fdcan_bus_t *bus = find_bus(hfdcan);
    if (bus == NULL || len > 8) return -1;

    FDCAN_TxHeaderTypeDef hdr = {
        .Identifier          = id,
        .IdType              = id_type,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = k_dlc[len],
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,     /* DJI/达妙 都是经典 CAN，不能开 BRS */
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };

    uint8_t buf[8] = {0};
    if (len) memcpy(buf, data, len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &hdr, buf) != HAL_OK) {
        bus->stat.tx_fail_cnt++;
        return -1;
    }
    bus->stat.tx_cnt++;
    return 0;
}

int bsp_fdcan_send_std(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data, uint8_t len)
{
    return send_frame(hfdcan, id & 0x7FFu, FDCAN_STANDARD_ID, data, len);
}

int bsp_fdcan_send_ext(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t *data, uint8_t len)
{
    return send_frame(hfdcan, id & 0x1FFFFFFFu, FDCAN_EXTENDED_ID, data, len);
}

const bsp_fdcan_stat_t *bsp_fdcan_get_stat(FDCAN_HandleTypeDef *hfdcan)
{
    fdcan_bus_t *bus = find_bus(hfdcan);
    return bus ? &bus->stat : NULL;
}

static uint8_t dlc_to_len(uint32_t dlc)
{
    for (uint8_t i = 0; i <= 8; i++) {
        if (k_dlc[i] == dlc) return i;
    }
    return 8;
}

/* HAL 回调：FIFO0 有新帧。一次把 FIFO 里的帧全部读完（例程只读一帧，负载高时会积压）。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    fdcan_bus_t *bus = find_bus(hfdcan);
    if (bus == NULL) return;

    FDCAN_RxHeaderTypeDef hdr;
    can_frame_t f;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, f.data) != HAL_OK) break;
        if (hdr.RxFrameType != FDCAN_DATA_FRAME) continue;

        f.id     = hdr.Identifier;
        f.is_ext = (hdr.IdType == FDCAN_EXTENDED_ID);
        f.len    = dlc_to_len(hdr.DataLength);
        bus->stat.rx_cnt++;

        for (int i = 0; i < bus->cb_num; i++) {
            if (bus->cb[i](hfdcan, &f)) break;
        }
    }
}

/* bus-off 自动恢复：H7 的 FDCAN 进入 bus-off 后 INIT 位被置 1，清掉才会重新上线。
 * 常见原因：CAN 线没接/没终端电阻/波特率不对，此时 bus_off_cnt 会一直涨。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        fdcan_bus_t *bus = find_bus(hfdcan);
        if (bus) bus->stat.bus_off_cnt++;
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}
