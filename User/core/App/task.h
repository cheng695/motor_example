#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdint.h>

#define DT7_RX_FRAME_SIZE 18U
extern uint8_t remote_rx_buffer[DT7_RX_FRAME_SIZE];

typedef struct
{
    float target_gimbal_yaw;
    float target_gimbal_pitch;
}Target;

typedef struct
{
    float out_gimbal_yaw;
    float out_gimbal_pitch;
}Output;

/* 外设初始化后调用一次，随后由 TIM1 更新中断调用 TaskUpdate()。 */
void Task_Init(void);
void TaskUpdate(void);

/* 接收中断使用的接口。 */
void Task_MotorFeedback(uint32_t can_id, const uint8_t data[8]);


#endif /* APP_TASK_H */
