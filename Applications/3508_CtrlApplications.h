/**
*@file 3508_CtrlApplications.h
*@brief C620 轮电机控制帧与反馈数据接口
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date
*
*控制帧 CAN ID 0x200；左、右轮反馈 ID 分别为 0x201、0x202。
*反馈帧 8 字节: ecd[0..1] 编码器原始值 [0~8191], speed_rpm[2..3] 转速 [rpm],
*given_current[4..5] 电流原始值, temperate[6] 温度 [℃]。
*motor_measure_t.last_ecd 保存上一周期 ecd，用于统计圈数。
**/
#ifndef __3508_CTRLAPPLICATIONS_H
#define __3508_CTRLAPPLICATIONS_H

#include "struct_typedef.h"
#include "fdcan.h"
#include <string.h>

#define CTRL_ID 		0x200	// 力矩控制帧 ID
#define CAN_3508_ML_ID  0x201	// 左轮反馈报文 ID
#define CAN_3508_MR_ID  0x202	// 右轮反馈报文 ID

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

extern motor_measure_t motor_measure_L;   // 左轮反馈数据
extern motor_measure_t motor_measure_R;   // 右轮反馈数据

/* 仅作诊断记录，不参与控制；指令与反馈均保留 C620 原始符号约定
 * current_cmd_raw[2]  最近一次下发的原始电流指令, [0] 左轮、[1] 右轮
 * tx_status           最近一次 FDCAN 发送返回状态
 * tx_cnt              发送成功累计次数
 * tx_fail_cnt         发送失败累计次数
 * last_cmd_tick       最近一次发送成功时刻 [ms]
 * last_fb_tick[2]     左、右轮最近一次反馈时刻 [ms]
 * rx_cnt[2]           左、右轮反馈帧累计计数
 */
typedef struct
{
    int16_t current_cmd_raw[2];
    uint32_t tx_status, tx_cnt, tx_fail_cnt, last_cmd_tick;
    uint32_t last_fb_tick[2], rx_cnt[2];
} wheel_drive_diag_t;
extern volatile wheel_drive_diag_t wheel_drive_diag;

void WheelTorque_Send(FDCAN_HandleTypeDef *hfdcan, float T_left, float T_right);

#endif
