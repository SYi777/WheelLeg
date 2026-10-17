/**
*@file Remote_control.h
*@brief 遥控器 SBUS 数据结构、协议常量与接口声明
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date
*
*接收解析与掉线检测实现在 Remote_control.c。
**/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_RC.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

#define SBUS_RX_BUF_NUM 36u	// SBUS 双缓冲接收区大小[字节]
#define RC_FRAME_LENGTH 18u	// 单帧长度[字节]
#define RC_CHANNAL_ERROR_VALUE 700 // 通道错误判定阈值

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

// 拨杆定义
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
// 键盘按键位定义
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
// 数据结构
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;
// 全局变量声明
extern RC_ctrl_t remote_ctrl;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
// 函数声明
/**
 * @brief  获取遥控器数据指针
 * @return 指向全局遥控器数据结构的常量指针
 */
const RC_ctrl_t *get_remote_control_point(void);
/**
 * @brief  判断遥控器数据是否异常
 * @return 0 数据正常；1 数据异常，所有控制量已清零
 *
 * 通道绝对值超过 RC_CHANNAL_ERROR_VALUE 或拨杆值为 0 时判为异常，
 * 异常时拨杆置为 RC_SW_DOWN。
 */
uint8_t RC_data_is_error(void);
/**
 * @brief  遥控器数据异常时重启接收
 * @return 无
 *
 * 调用 RC_restart 重启 UART5 DMA 双缓冲接收。
 */
void slove_data_error(void);
/**
 * @brief  遥控器模块底层初始化
 * @return 无
 *
 * 配置 UART5 DMA 双缓冲接收，缓冲区为 SBUS_MultiRx_Buf。
 */
void RemoteContrlInit(void);
/**
 * @brief  遥控器掉线检测，需 1kHz 周期调用
 * @return 无
 *
 * flag 由接收中断置 1；连续约 100 ms 未收到 SBUS 帧时将 rc_online 置 0，
 * 恢复收帧后自动置 1。
 */
void RC_OnlineCheck(void);

#endif
