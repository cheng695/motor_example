#include "user_uart.h"
#include <stddef.h>

#define UART_BYTE_TIMEOUT_MS 100U

bool Transmit(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Transmit_IT(huart, buffer, size) == HAL_OK;
}

bool Receive(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Receive_IT(huart, buffer, size) == HAL_OK;
}

bool Transmit_byte(UART_HandleTypeDef *huart, uint8_t byte)
{
    if (huart == NULL)
        return false;

    return HAL_UART_Transmit(huart, &byte, 1U, UART_BYTE_TIMEOUT_MS) == HAL_OK;
}

bool Receive_byte(UART_HandleTypeDef *huart, uint8_t *byte)
{
    if (huart == NULL || byte == NULL)
        return false;

    return HAL_UART_Receive(huart, byte, 1U, UART_BYTE_TIMEOUT_MS) == HAL_OK;
}

bool Transmit_dma(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || huart->hdmatx == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Transmit_DMA(huart, buffer, size) == HAL_OK;
}

bool Receive_dma(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || huart->hdmarx == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Receive_DMA(huart, buffer, size) == HAL_OK;
}

bool Receive_dma_idle(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || huart->hdmarx == NULL || buffer == NULL || size == 0U)
    {
        return false;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size) != HAL_OK)
    {
        return false;
    }

    /* 由 DMA 搬运接收数据，关闭逐字节接收中断。 */
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

    /* 只处理空闲或接收完成事件，关闭 DMA 半传输中断。 */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    return true;
}

