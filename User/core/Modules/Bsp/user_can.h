#ifndef USER_CAN_H_
#define USER_CAN_H_

#include <stdint.h>
#include "can.h"


typedef struct
{
    CAN_RxHeaderTypeDef rxHeader; /**< 接收帧头，包含报文 ID、帧类型和数据长度。 */
    uint8_t rxdata[8];            /**< 接收数据，有效字节数由帧头的 DLC 指定。 */
} Can_RX;
extern Can_RX Can_RX_;


void CAN_ConfigFilter(void);
void sendCAN(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef canheader,
             uint8_t tx_data[8], uint32_t sendCAN_ID, uint32_t txMailbox);

#endif /* USER_CAN_H_ */
