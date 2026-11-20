#ifndef USER_UART_H
#define USER_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "usart.h"

/* 中断收发。返回 true 表示已启动，缓冲区须保留到传输完成。 */
bool Transmit(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size);
bool Receive(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

/* 阻塞收发一个字节，超时时间为 100 ms。 */
bool Transmit_byte(UART_HandleTypeDef *huart, uint8_t byte);
bool Receive_byte(UART_HandleTypeDef *huart, uint8_t *byte);

/* DMA 收发。返回 true 表示已启动，缓冲区须保留到传输完成。 */
bool Transmit_dma(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size);
bool Receive_dma(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

/* DMA 接收，串口空闲或缓冲区接满时上报接收事件。 */
bool Receive_dma_idle(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);

#endif /* USER_UART_H */
