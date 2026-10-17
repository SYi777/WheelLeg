/**
*@file DM_motor.h
*@brief 达妙 DM-J8009P-2EC 电机驱动
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-3
*
*通信协议：标准 CAN，1 Mbps
*控制帧 ID = 电机 CAN ID
*反馈帧 ID = MASTER ID(0)
*！！！！逆时针为正！！！！
*
*MIT 控制帧格式：
* D[0] : p_des(high 8)
* D[1] : p_des(low  8)
* D[2] : v_des(high 8)
* D[3] : v_des(low  4) + Kp(high 4)
* D[4] : Kp(low  8)
* D[5] : Kd(high 8)
* D[6] : Kd(low  4) + t_ff(high 4)
* D[7] : t_ff(low  8)
*
*反馈帧格式：
*
* D[0] : ERR(normal=0)+ID(后4位)
* D[1] : POS(high 8)
* D[2] : POS(low  8)
* D[3] : VEL(high 8)
* D[4] : VEL(low  4) + T(high 4)
* D[5] : T(扭矩)
* D[6] : T_MOS
* D[7] : T_Rotor
*
**/
#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H
#include "fdcan.h"
#include <string.h>

#define DM_ENCODER_BITS 14
#define DM_ENCODER_RESOLUTION (1 << DM_ENCODER_BITS) // 编码器分辨率 16384
// MIT 控制量范围
#define DM_POS_MIN (-3.1415926f)
#define DM_POS_MAX (3.1415926f)
#define DM_VEL_MIN (-45.0f)
#define DM_VEL_MAX (45.0f)
#define DM_TOR_MIN (-54.0f)
#define DM_TOR_MAX (54.0f)
#define DM_KP_MIN	0.0f
#define DM_KP_MAX	500.0f
#define DM_KD_MIN	0.0f
#define DM_KD_MAX	5.0f
// 通信超时
#define DM_COMM_TIMEOUT_MS 200
// 故障码
typedef enum
{
    DM_FAULT_NONE        = 0x0,   // 无故障
    DM_FAULT_OVERVOLT    = 0x8,   // 超压
    DM_FAULT_UNDERVOLT   = 0x9,   // 欠压
    DM_FAULT_OVERCURR    = 0xA,   // 过电流
    DM_FAULT_MOS_OVERT   = 0xB,   // MOS 过温
    DM_FAULT_COIL_OVERT  = 0xC,   // 线圈过温
    DM_FAULT_COMM_LOSS   = 0xD,   // 通讯丢失
    DM_FAULT_OVERLOAD    = 0xE,   // 过载
}DM_FaultCode;
// 电机状态
typedef struct
{
	// 原始值
	uint16_t pos_raw;
	uint16_t vel_raw;
	uint16_t tor_raw;
	uint8_t err;
	uint8_t motor_id;
	uint8_t t_mos;
	uint8_t t_rotor;
	// 物理量
	float position;		// 位置 [rad]
	float velocity;		// 速度 [rad/s]
	float torque;		// 力矩 [N*m]
	float processed_pos;// 处理后绝对角度 [rad]，范围 0~2π，便于位置环
	// 状态标志
	uint8_t enabled;	// 使能标志，err == 1
	uint8_t fault;		// 故障标志，err > 1
	uint8_t fault_code;
}DM_state;
// 控制命令
typedef struct {
    // 量化值
    uint16_t  p_des;         // 目标位置量化值
    uint16_t  v_des;         // 目标速度量化值
    uint16_t kp;            // 位置增益 Kp 量化值
    uint16_t kd;            // 速度增益 Kd 量化值
    uint16_t  t_ff;          // 前馈力矩量化值
} DM_Command;
typedef struct {
    uint8_t      can_id;           // 电机 CAN ID
    DM_Command   cmd;              // 待发送命令
    DM_state	 fb;               // 最新反馈
    uint32_t     last_fb_tick;     // 上次收到反馈的时间戳 [ms]
    uint32_t     last_cmd_tick;    // 上次发送命令的时间戳 [ms]
    uint8_t      online;           // 在线标志
    uint32_t     rx_cnt;           // 收包计数
    uint32_t     tx_cnt;           // 发包计数
    HAL_StatusTypeDef last_tx_status; // 上次发送结果
} DM_Motor;

void DM_PackMITCmd(const DM_Command *cmd, uint8_t data[8]);
void DM_UnpackFb(const uint8_t data[8], DM_state *fb);
HAL_StatusTypeDef DM_SendCmd(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m);

// 电机使能
HAL_StatusTypeDef DM_Enable(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m);
// 电机关闭
void DM_Disable(FDCAN_HandleTypeDef *hfdcan, DM_Motor *m);
// MIT 控制
void DM_MITCtrl(DM_Motor *m, float pos, float vel, float kp, float kd, float t_ff);
// 停转
void DM_CtrlStop(DM_Motor *m);

// 工具函数：浮点与无符号整数互转
static inline int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (int)((x_float - x_min) * ((float)((1 << bits) - 1)) / span);
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}
#endif
