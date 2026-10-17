//参考文献：王草凡知乎文章
//将C板代码优化并移植到H7，采用了库函数对DMA进行初始化，避开了寄存器操作，整体逻辑和C板一样
#include "Remote_control.h"

RC_ctrl_t remote_ctrl;//遥控器控制变量
__attribute__ ((section(".ARM.__at_0x24000000"))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
//内部函数
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static int16_t RC_abs(int16_t value);
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);
//---------------------------------------------------------------------
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     remote_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     remote_ctrl: 遥控器数据指
  * @retval         none
  */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL)
    {
        return;
    }

    remote_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    remote_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    remote_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    remote_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    remote_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    remote_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
 * @brief  取绝对值。
 * @param  value 有符号值。
 * @retval 绝对值。
 */
static int16_t RC_abs(int16_t value)
{
    return (value > 0) ? value : -value;
}

/**
 * @brief  UART5 接收事件处理函数。
 *         根据当前 DMA 目标缓冲区（CT 位）处理接收到的数据。
 * @param  huart 串口句柄。
 * @param  Size  本次接收到的字节数。
 */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        /* 缓冲区0 接收完成 */
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
        /* 缓冲区1 接收完成 */
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
 * @brief  串口空闲中断回调函数。
 *         当 UART5 触发空闲中断时，调用实际处理函数。
 * @param  huart 串口句柄。
 * @param  Size  接收到的数据字节数。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart5)
    {
        USER_USART5_RxHandler(huart, Size);
    }
}
/**
 * @brief  获取遥控器数据指针。
 * @retval 指向全局遥控器数据结构的常量指针。
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &remote_ctrl;
}

/**
 * @brief  判断当前遥控器数据是否存在错误。
 *         当任一摇杆通道绝对值超过阈值，或任一拨杆值为0时视为错误。
 *         错误时将各通道置零、拨杆置为最低、鼠标与键盘数据清零。
 * @retval 0: 数据正常；1: 发生错误。
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
/**
 * @brief  数据错误后的恢复处理。
 *         调用 RC_restart 重启 DMA 双缓冲接收。
 */
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM, SBUS_MultiRx_Buf[0], SBUS_MultiRx_Buf[1]);
}
void RemoteContrlInit(void)
{
	//开启DMA转发，数据走双缓冲进入内存
	USART_RxDMA_DoubleBuffer_Init(&huart5,(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
}
