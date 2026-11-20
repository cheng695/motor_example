#include "task.h"
#include "DT7.h"
#include "user_can.h"
#include "user_uart.h"
#include "tim.h"
#include "usart.h"

uint8_t remote_rx_buffer[DT7_RX_FRAME_SIZE];

/* TIM1 每 1 ms 溢出一次，在更新中断中运行控制任务。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        TaskUpdate();
    }
}

/********************************* can回调 ********************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1)
        return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_RX_.rxHeader, Can_RX_.rxdata) != HAL_OK)
        return;

    if (Can_RX_.rxHeader.IDE == CAN_ID_STD &&
        Can_RX_.rxHeader.RTR == CAN_RTR_DATA &&
        Can_RX_.rxHeader.DLC == 8)
    {
        Task_MotorFeedback(Can_RX_.rxHeader.StdId, Can_RX_.rxdata);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan2)
        return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_RX_.rxHeader, Can_RX_.rxdata) != HAL_OK)
        return;

    if (Can_RX_.rxHeader.IDE == CAN_ID_STD &&
        Can_RX_.rxHeader.RTR == CAN_RTR_DATA &&
        Can_RX_.rxHeader.DLC == 8)
    {
        Task_MotorFeedback(Can_RX_.rxHeader.StdId, Can_RX_.rxdata);
    }
}




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart1)
        return;

    if (size == sizeof(remote_rx_buffer))
    {
        DT7_Update(remote_rx_buffer);
    }

    if (!Receive_dma_idle(huart, remote_rx_buffer, sizeof(remote_rx_buffer)))
    {
        Error_Handler();
    }
}


