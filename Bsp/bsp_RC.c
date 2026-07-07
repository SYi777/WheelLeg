#include "bsp_RC.h"

/**
 * @brief  初始化 UART DMA 双缓冲接收。
 * @param  huart             串口句柄。
 * @param  DstAddress        第一个缓冲区地址。
 * @param  SecondMemAddress  第二个缓冲区地址。
 * @param  DataLength        总传输字节数（两帧之和）。
 */
void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart,
                                   uint32_t *DstAddress,
                                   uint32_t *SecondMemAddress,
                                   uint32_t DataLength)
{
    /* 配置接收类型为空闲中断模式 */
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType   = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize    = DataLength;
    /* 使能 UART DMA 接收请求 */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    /* 使能 UART 空闲中断 */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    /* 启动 DMA 双缓冲模式 */
    HAL_DMAEx_MultiBufferStart(huart->hdmarx,
                               (uint32_t)&huart->Instance->RDR,
                               (uint32_t)DstAddress,
                               (uint32_t)SecondMemAddress,
                               DataLength);
}
/**
 * @brief  重启 DMA 双缓冲接收。
 *         重新配置外设地址、缓冲区地址、传输长度并确保从 Buffer0 开始。
 * @param  dma_buf_num 传输数据项总数（例如 36）。
 * @param  Buffer1     缓冲区0地址。
 * @param  Buffer2     缓冲区1地址。
 */
void RC_restart(uint16_t dma_buf_num,uint8_t *Buffer1,uint8_t *Buffer2)
{
    //关闭DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    //等待DMA真正停止
    while (((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_EN);
    //重新配置 DMA 双缓冲的核心寄存器
    DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)hdma_uart5_rx.Instance;
    dma->PAR  = (uint32_t)&huart5.Instance->RDR;        // 外设地址 = 串口接收数据寄存器
    dma->M0AR = (uint32_t)Buffer1;          			// 缓冲区0
    dma->M1AR = (uint32_t)Buffer2;          			// 缓冲区1
    dma->NDTR = dma_buf_num;                            // 传输总数 = RC_FRAME_LENGTH * 2
    //确保双缓冲模式使能，且从 Buffer0 开始
    dma->CR |= DMA_SxCR_DBM;        // 置位 DBM（双缓冲模式）
    dma->CR &= ~DMA_SxCR_CT;        // 清零 CT，从 M0（Buffer0）开始接收
    //重新使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);
}
