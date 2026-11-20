#ifndef APP_FSM_H
#define APP_FSM_H

typedef enum
{
    FSM_STATE_STOP,
    FSM_STATE_MANUAL,
    FSM_STATE_VISION,
} FsmState_t;

typedef enum
{
    FSM_EVENT_STOP,
    FSM_EVENT_ENTER_MANUAL,
    FSM_EVENT_ENTER_VISION,
} FsmEvent_t;

/* 将状态机重置为 STOP。 */
void FSM_Init(void);

/* 读取遥控器拨杆，产生并处理模式切换事件。 */
void FSM_Update(void);

/* 根据事件切换状态。 */
void FSM_HandleEvent(FsmEvent_t event);

FsmState_t FSM_GetState(void);

#endif /* APP_FSM_H */
