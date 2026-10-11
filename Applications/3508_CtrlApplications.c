#include "3508_CtrlApplications.h"

volatile wheel_drive_diag_t wheel_drive_diag;

motor_measure_t motor_measure_L;   // ×óÂÖ
motor_measure_t motor_measure_R;   // ÓÒÂÖ

//ÂÖì±Å¤¾ØÏÂ·¢
void WheelTorque_Send(FDCAN_HandleTypeDef *hfdcan, float T_left, float T_right)
{
    int16_t cur[4];
    cur[0] = (int16_t)(T_left*3000);   // ×óÂÖ
    cur[1] = (int16_t)(-T_right*3000);   // ÓÒÂÖ
    cur[2] = 0;
    cur[3] = 0;

    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
    for (int i = 0; i < 4; i++)          // C620 0x200: high byte first, then low byte
    {
        uint16_t raw = (uint16_t)cur[i];
        data[2*i]     = (uint8_t)(raw >> 8);
        data[2*i + 1] = (uint8_t)(raw & 0xFF);
    }
    tx_header.Identifier          = CTRL_ID;                 // 0x200
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;
    wheel_drive_diag.current_cmd_raw[0] = cur[0];
    wheel_drive_diag.current_cmd_raw[1] = cur[1];
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
    wheel_drive_diag.tx_status = (uint32_t)status;
    if (status == HAL_OK)
    {
        wheel_drive_diag.tx_cnt++;
        wheel_drive_diag.last_cmd_tick = HAL_GetTick();
    }
    else wheel_drive_diag.tx_fail_cnt++;
}
