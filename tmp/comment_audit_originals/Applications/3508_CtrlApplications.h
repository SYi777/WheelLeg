/**
*@file 3508_CtrlApplications.h
*@brief C620轮电机控制帧与反馈数据接口
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date
*
*控制帧 CAN ID 0x200；左、右轮反馈 ID 分别为 0x201、0x202。
**/
#ifndef __3508_CTRLAPPLICATIONS_H
#define __3508_CTRLAPPLICATIONS_H

#include "struct_typedef.h"
#include "fdcan.h"
#include <string.h>

#define CTRL_ID 		0x200	//力矩控制帧id
#define CAN_3508_ML_ID  0x201	//左腿回馈报文id
#define CAN_3508_MR_ID  0x202	//右腿回馈报文id

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

#define get_motor_measure(ptr, data)                               \
{                                                                  \
	(ptr)->last_ecd = (ptr)->ecd;                                  \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
	(ptr)->temperate = (data)[6];                                  \
}

extern motor_measure_t motor_measure_L;   // 左轮
extern motor_measure_t motor_measure_R;   // 右轮

/* Diagnostic metadata only; feedback and command retain raw C620 signs. */
typedef struct
{
    int16_t current_cmd_raw[2];
    uint32_t tx_status, tx_cnt, tx_fail_cnt, last_cmd_tick;
    uint32_t last_fb_tick[2], rx_cnt[2];
} wheel_drive_diag_t;
extern volatile wheel_drive_diag_t wheel_drive_diag;

void WheelTorque_Send(FDCAN_HandleTypeDef *hfdcan, float T_left, float T_right);

#endif
