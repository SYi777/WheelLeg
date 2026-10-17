#ifndef __BSP_RC_H
#define __BSP_RC_H
#include "stm32h7xx_hal.h"
#include "struct_typedef.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/**
 * @brief  初始化 UART DMA 双缓冲接收。
 *         配置串口为空闲中断模式，启动 DMA 将数据搬运到双缓冲区。
 * @param  huart             串口句柄。
 * @param  DstAddress        第一个缓冲区地址。
 * @param  SecondMemAddress  第二个缓冲区地址。
 * @param  DataLength        总传输字节数（两帧之和）。
 */
void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart,
                                   uint32_t *DstAddress,
                                   uint32_t *SecondMemAddress,
                                   uint32_t DataLength);
/**
 * @brief  重启 DMA 双缓冲接收（例如在数据错误后重新开始）。
 *         关闭 DMA 并将各关键寄存器恢复为初始值。
 * @param  dma_buf_num 传输数据项总数（两帧之和）。
 * @param  Buffer1     缓冲区0地址。
 * @param  Buffer2     缓冲区1地址。
 */
void RC_restart(uint16_t dma_buf_num, uint8_t *Buffer1, uint8_t *Buffer2);

#endif
