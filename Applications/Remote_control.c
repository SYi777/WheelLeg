/**
*@file Remote_control.c
*@brief 遥控器 SBUS 数据接收、解析与掉线检测
*@Author 王草凡 SYi777
*@Email jyh1621839418@163.com
*@Date 2026-10-13
*
*参考王草凡的 C 板实现，移植到 H7 并使用 HAL 初始化 DMA，增加遥控器离线保护。
**/
// SBUS 协议说明参考王草凡的遥控器实现
// 解析逻辑移植自 C 板，DMA 由 HAL 初始化
#include "Remote_control.h"
#include "struct_typedef.h"

volatile uint8_t rc_online = 0;			// 遥控器在线标志[0 离线, 1 在线]
volatile uint8_t flag      = 0;			// 新帧标志[1 表示收到新帧]
volatile uint32_t error_count = 0;		// 连续无帧计数[ms]

RC_ctrl_t remote_ctrl;// 遥控器数据
__attribute__ ((section(".ARM.__at_0x24000000"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];// SBUS 双缓冲接收区，固定于 0x24000000，防止 DMA 越界
// 内部函数声明
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static int16_t RC_abs(int16_t value);
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);
/**
 * @brief  解析一帧 SBUS 数据并更新遥控器状态
 * @param  sbus_buf     收到的 SBUS 原始数据
 * @param  remote_ctrl  输出的遥控器状态
 * @return 无
 *
 * 各通道为 11 位原始值，解析后统一扣除 RC_CH_VALUE_OFFSET 零偏。
 */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL)
    {
        return;
    }

    remote_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        // 通道 0
    remote_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; // 通道 1
    remote_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          // 通道 2
                         (sbus_buf[4] << 10)) &0x07ff;
    remote_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; // 通道 3
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  		// 左拨杆
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  // 右拨杆
    remote_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    // 鼠标 X 轴
    remote_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    // 鼠标 Y 轴
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  // 鼠标 Z 轴
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  // 鼠标左键
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  // 鼠标右键
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    // 键盘按键
    remote_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // 通道 4

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

// 求通道值的绝对值
static int16_t RC_abs(int16_t value)
{
    return (value > 0) ? value : -value;
}

/**
 * @brief  处理 UART5 DMA 双缓冲接收事件
 * @param  huart  UART 句柄
 * @param  Size   本次接收的字节数
 * @return 无
 *
 * 根据 DMA 目标缓冲区位判断接收完成的是哪个缓冲区，
 * 仅在长度等于 RC_FRAME_LENGTH 时解析。
 */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    flag = 1;   // 收到接收事件，喂狗
	
	
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        // 缓冲区 0 接收完成
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);
        if (Size == RC_FRAME_LENGTH)
        {
            SBUS_TO_RC(SBUS_MultiRx_Buf[0], &remote_ctrl);
        }
    }
    else
    {
        // 缓冲区 1 接收完成
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);
        if (Size == RC_FRAME_LENGTH)
        {
            SBUS_TO_RC(SBUS_MultiRx_Buf[1], &remote_ctrl);
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief  将 UART5 接收事件转交给遥控器处理函数
 * @param  huart  触发事件的 UART 句柄
 * @param  Size   本次接收的字节数
 * @return 无
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart5)
    {
        USER_USART5_RxHandler(huart, Size);
    }
}
// 获取全局遥控器数据指针
const RC_ctrl_t *get_remote_control_point(void)
{
    return &remote_ctrl;
}

/**
 * @brief  检测遥控器通道与拨杆数据是否异常
 * @return 0 正常；1 异常，且遥控器状态已清零
 *
 * 任一通道绝对值超过 RC_CHANNAL_ERROR_VALUE 或拨杆值为 0 即判为异常，
 * 异常时将遥控器数据全部清零，拨杆置为 RC_SW_DOWN。
 */
uint8_t RC_data_is_error(void)
{
    if (RC_abs(remote_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(remote_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(remote_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(remote_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (remote_ctrl.rc.s[0] == 0) goto error;
    if (remote_ctrl.rc.s[1] == 0) goto error;
    return 0;
error:
    remote_ctrl.rc.ch[0] = 0;
    remote_ctrl.rc.ch[1] = 0;
    remote_ctrl.rc.ch[2] = 0;
    remote_ctrl.rc.ch[3] = 0;
    remote_ctrl.rc.ch[4] = 0;
    remote_ctrl.rc.s[0] = RC_SW_DOWN;
    remote_ctrl.rc.s[1] = RC_SW_DOWN;
    remote_ctrl.mouse.x = 0;
    remote_ctrl.mouse.y = 0;
    remote_ctrl.mouse.z = 0;
    remote_ctrl.mouse.press_l = 0;
    remote_ctrl.mouse.press_r = 0;
    remote_ctrl.key.v = 0;
    return 1;
}
// 重启 UART5 DMA 双缓冲接收
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM, SBUS_MultiRx_Buf[0], SBUS_MultiRx_Buf[1]);
}
// 初始化 UART5 DMA 双缓冲接收
void RemoteContrlInit(void)
{
	// 启动 DMA 双缓冲接收
	USART_RxDMA_DoubleBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
}

volatile uint8_t* GetRCOnlineFlag(void)
{
	return &rc_online;
}

/**
 * @brief  遥控器掉线检测，需 1kHz 周期调用
 * @return 无
 *
 * flag 由接收中断置 1，本函数由左腿任务按 1 kHz 调用（全车唯一调用点）；
 * 连续约 100 ms 未收到帧时清除 rc_online，恢复收帧后自动置 1。
 */
void RC_OnlineCheck(void)
{
    if (flag)
    {
        flag = 0;               // 清标志，等待下一帧
        rc_online = 1;          // 有帧，判为在线
        error_count = 0;
    }
    else
    {
        if (error_count < 100U) error_count++;   // 每毫秒加一，封顶防溢出
        if (error_count >= 100U) rc_online = 0;  // 约 100 ms 无帧判掉线
    }
}
