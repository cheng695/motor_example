#include "user_uart.h"
#include <stddef.h>

#define UART_BYTE_TIMEOUT_MS 100U // 单字节阻塞收发的超时时间，单位 ms

/********************************* 中断收发 ********************************************/

/**
 * @brief 启动串口中断发送
 * @param huart 串口句柄
 * @param buffer 发送数据缓冲区，发送完成前不能修改或释放
 * @param size 发送数据长度，单位字节
 * @return 启动成功返回 true，参数无效或串口忙等情况返回 false
 */
bool Transmit(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Transmit_IT(huart, buffer, size) == HAL_OK;
}

/**
 * @brief 启动串口中断接收
 * @param huart 串口句柄
 * @param buffer 接收数据缓冲区，接收完成前不能释放
 * @param size 接收数据长度，单位字节
 * @return 启动成功返回 true，参数无效或串口忙等情况返回 false
 */
bool Receive(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Receive_IT(huart, buffer, size) == HAL_OK;
}

/********************************* 单字节收发 ********************************************/

/**
 * @brief 阻塞发送一个字节，最多等待 UART_BYTE_TIMEOUT_MS
 * @param huart 串口句柄
 * @param byte 待发送的字节
 * @return 发送成功返回 true，参数无效、超时或发送失败返回 false
 */
bool Transmit_byte(UART_HandleTypeDef *huart, uint8_t byte)
{
    if (huart == NULL)
        return false;

    return HAL_UART_Transmit(huart, &byte, 1U, UART_BYTE_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief 阻塞接收一个字节，最多等待 UART_BYTE_TIMEOUT_MS
 * @param huart 串口句柄
 * @param byte 接收字节的存放地址
 * @return 接收成功返回 true，参数无效、超时或接收失败返回 false
 */
bool Receive_byte(UART_HandleTypeDef *huart, uint8_t *byte)
{
    if (huart == NULL || byte == NULL)
        return false;

    return HAL_UART_Receive(huart, byte, 1U, UART_BYTE_TIMEOUT_MS) == HAL_OK;
}

/********************************* DMA收发 ********************************************/

/**
 * @brief 启动串口DMA发送
 * @param huart 串口句柄，需要先配置发送DMA
 * @param buffer 发送数据缓冲区，发送完成前不能修改或释放
 * @param size 发送数据长度，单位字节
 * @return 启动成功返回 true，参数无效、未配置DMA或启动失败返回 false
 */
bool Transmit_dma(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || huart->hdmatx == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Transmit_DMA(huart, buffer, size) == HAL_OK;
}

/**
 * @brief 启动串口DMA接收
 * @param huart 串口句柄，需要先配置接收DMA
 * @param buffer 接收数据缓冲区，接收完成前不能释放
 * @param size 接收数据长度，单位字节
 * @return 启动成功返回 true，参数无效、未配置DMA或启动失败返回 false
 */
bool Receive_dma(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    if (huart == NULL || huart->hdmarx == NULL || buffer == NULL || size == 0U)
        return false;

    return HAL_UART_Receive_DMA(huart, buffer, size) == HAL_OK;
}

/**
 * @brief 启动串口DMA空闲接收，在空闲或接收完成时进入接收事件回调
 * @param huart 串口句柄，需要先配置接收DMA
 * @param buffer 接收数据缓冲区，接收完成前不能释放
 * @param size 接收缓冲区容量，单位字节
 * @return 启动成功返回 true，参数无效、未配置DMA或启动失败返回 false
 */
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

