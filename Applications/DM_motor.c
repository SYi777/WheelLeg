/**
*@file DM_motor.c
*@brief 达妙DM-J8009P-2EC电机驱动
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-4
**/
#include "DM_motor.h"
#include "8009_CtrlApplications.h"

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            //根据反馈ID匹配电机（J8009反馈ID = 0x10 + 电机ID）
            switch(RxHeader.Identifier)
            {
                case 0x11: //左前电机（ID=1）
                    DM_UnpackFb(RxData, &DM8009_Ctrl[0].fb);
                    DM8009_Ctrl[0].online = 1;
                    DM8009_Ctrl[0].last_fb_tick = HAL_GetTick();
                    DM8009_Ctrl[0].rx_cnt++;
                    break;
                    
                case 0x12: //左后电机（ID=2）
                    DM_UnpackFb(RxData, &DM8009_Ctrl[1].fb);
                    DM8009_Ctrl[1].online = 1;
                    DM8009_Ctrl[1].last_fb_tick = HAL_GetTick();
                    DM8009_Ctrl[1].rx_cnt++;
                    break;
                    
                case 0x13: //右前电机（ID=3）
                    DM_UnpackFb(RxData, &DM8009_Ctrl[2].fb);
                    DM8009_Ctrl[2].online = 1;
                    DM8009_Ctrl[2].last_fb_tick = HAL_GetTick();
                    DM8009_Ctrl[2].rx_cnt++;
                    break;
                    
                case 0x14: //右后电机（ID=4）
                    DM_UnpackFb(RxData, &DM8009_Ctrl[3].fb);
                    DM8009_Ctrl[3].online = 1;
                    DM8009_Ctrl[3].last_fb_tick = HAL_GetTick();
                    DM8009_Ctrl[3].rx_cnt++;
                    break;
                    
                default:
                    break;
            }
        }
    }
}
//MIT命令打包
void DM_PackMITCmd(const DM_Command *cmd, uint8_t data[8])
{
    uint16_t p  = cmd->p_des;
    uint16_t v  = cmd->v_des & 0x0FFF;
    uint16_t kp = cmd->kp    & 0x0FFF;
    uint16_t kd = cmd->kd    & 0x0FFF;
    uint16_t t  = cmd->t_ff  & 0x0FFF;

    // D0 = p_des[15:8] 高字节
    data[0] = (p >> 8) & 0xFF;
    // D1 = p_des[7:0] 低字节
    data[1] = p & 0xFF;

    // D2 = v_des[11:4]
    data[2] = (v >> 4) & 0xFF;
    // D3 = v_des[3:0] <<4 | Kp[11:8]
    data[3] = ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F);

    // D4 = Kp[7:0]
    data[4] = kp & 0xFF;

    // D5 = Kd[11:4]
    data[5] = (kd >> 4) & 0xFF;
    // D6 = Kd[3:0] <<4 | t_ff[11:8]
    data[6] = ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F);
    // D7 = t_ff[7:0]
    data[7] = t & 0xFF;
}
//解包
void DM_UnpackFb(const uint8_t data[8], DM_state *fb)
{
    fb->motor_id = data[0] & 0x0F;
    fb->err      = data[0] >> 4;
    fb->enabled    = (fb->err == 1) ? 1 : 0;
    fb->fault_code = (DM_FaultCode)fb->err;
    fb->fault      = (fb->err > 0x01) ? 1 : 0;

    fb->pos_raw = (uint16_t)((data[1] << 8) | data[2]);
    fb->vel_raw = (uint16_t)((data[3] << 4) | ((data[4] >> 4) & 0x0F));
    fb->tor_raw = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);

    fb->t_mos   = data[6];
    fb->t_rotor = data[7];

    fb->position = uint_to_float((int)fb->pos_raw, DM_POS_MIN, DM_POS_MAX, 16);
    fb->velocity = uint_to_float((int)fb->vel_raw, DM_VEL_MIN, DM_VEL_MAX, 12);
    fb->torque   = uint_to_float((int)fb->tor_raw, DM_TOR_MIN, DM_TOR_MAX, 12);
	
	if(fb->position < 0)
	{
		fb->processed_pos = fb->position + 2*3.1415926f;
	}
	else
	{
		fb->processed_pos = fb->position;
	}
}
// 控制帧发送函数，返回发送状态
HAL_StatusTypeDef DM_SendCmd(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m)
{
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t data[8];
	DM_PackMITCmd(&m->cmd,data);
	tx_header.Identifier          = m->can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;
	HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,data);
	if(st == HAL_OK)
	{
		m->tx_cnt++;
		m->last_cmd_tick = HAL_GetTick();
	}
	m->last_tx_status = st;
	return st;
}
//系统命令帧发送函数
static HAL_StatusTypeDef DM_SendSysCmd(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m,const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier          = m->can_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}
//使能电机
void DM_Enable(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    DM_SendSysCmd(hfdcan, m, data);
}
//失能电机
void DM_Disable(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    DM_SendSysCmd(hfdcan, m, data);
}
//MIT
void DM_MITCtrl(DM_Motor *m, float pos, float vel, float kp, float kd, float t_ff)
{
    if (kp < DM_KP_MIN) kp = DM_KP_MIN;
    if (kp > DM_KP_MAX) kp = DM_KP_MAX;
    if (kd < DM_KD_MIN) kd = DM_KD_MIN;
    if (kd > DM_KD_MAX) kd = DM_KD_MAX;
    if (t_ff > DM_TOR_MAX) t_ff = DM_TOR_MAX;
    if (t_ff < DM_TOR_MIN) t_ff = DM_TOR_MIN;
	
    m->cmd.p_des = float_to_uint(pos, DM_POS_MIN, DM_POS_MAX, 16);
    m->cmd.v_des = float_to_uint(vel, DM_VEL_MIN, DM_VEL_MAX, 12);
    m->cmd.kp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    m->cmd.kd = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    m->cmd.t_ff = float_to_uint(t_ff, DM_TOR_MIN, DM_TOR_MAX, 12);
}
//停转
void DM_CtrlStop(DM_Motor *m)
{
    m->cmd.p_des = float_to_uint(0.0f, DM_POS_MIN, DM_POS_MAX, 16);
    m->cmd.v_des = float_to_uint(0.0f, DM_VEL_MIN, DM_VEL_MAX, 12);
    m->cmd.kp = 0;
    m->cmd.kd = 0;
    m->cmd.t_ff = float_to_uint(0.0f, DM_TOR_MIN, DM_TOR_MAX, 12);
}
