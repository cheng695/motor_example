#include "user_can.h"
#include <stdint.h>

Can_RX Can_RX_;


/**
 * @brief CAN过滤器配置
 * 
 */
void CAN_ConfigFilter()//过滤函数，关闭屏蔽器
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;         //启动过滤器
    sFilterConfig.FilterBank           = 0;                         //使用过滤器0
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;          //缓冲器0 有0，1两个
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;     //掩码模式
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;     //处理标准ID和扩展ID
    sFilterConfig.FilterIdHigh         = 0x00;                      //以下四行表示接受所有ID
    sFilterConfig.FilterIdLow          = 0x00;
    sFilterConfig.FilterMaskIdHigh     = 0x00;
    sFilterConfig.FilterMaskIdLow      = 0x00;
    sFilterConfig.SlaveStartFilterBank = 14;                
    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 将一帧 CAN 数据提交到发送邮箱
 * @param hcan CAN 外设句柄，用于选择 CAN1 或 CAN2
 * @param canheader CAN 发送帧头，包含帧类型和数据长度等信息
 * @param tx_data 待发送的数据缓冲区，长度至少为帧头中的 DLC
 * @param sendCAN_ID 本帧使用的 CAN 报文标识符
 * @param txMailbox 输出参数，用于保存本次发送所使用的邮箱
 */
void sendCAN(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef canheader, uint8_t tx_data[8], uint32_t sendCAN_ID, uint32_t txMailbox)
{
    canheader.StdId = sendCAN_ID;     //ID号 标准标识符
    canheader.IDE = CAN_ID_STD;       //使用标准帧
    canheader.RTR = CAN_RTR_DATA;     //数据帧
    canheader.DLC = 8;                

    if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0)    
    {
        HAL_CAN_AddTxMessage(hcan, &canheader, tx_data, &txMailbox);
    }
}
