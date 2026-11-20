#include "fsm.h"
#include "DT7.h"
#include "main.h"

#define UP     1
#define MIDDLE 3
#define DOWN   2

static FsmState_t current_state = FSM_STATE_STOP;
static FsmEvent_t read_mode_event(void)
{
    int16_t left_switch  = get_s1();
    int16_t right_switch = get_s2();

    if (left_switch == MIDDLE && right_switch == DOWN)
        return FSM_EVENT_ENTER_MANUAL;

    if (left_switch == DOWN && right_switch == MIDDLE)
        return FSM_EVENT_ENTER_VISION;

    return FSM_EVENT_STOP;
}

void FSM_Update(void)
{
    FSM_HandleEvent(read_mode_event());
}

void FSM_Init(void)
{
    current_state = FSM_STATE_STOP;
}

void FSM_HandleEvent(FsmEvent_t event)
{
    switch (event)
    {
        case FSM_EVENT_STOP:
            current_state = FSM_STATE_STOP;
            break;

        case FSM_EVENT_ENTER_MANUAL:
            current_state = FSM_STATE_MANUAL;
            break;

        case FSM_EVENT_ENTER_VISION:
            current_state = FSM_STATE_VISION;
            break;

        default:
            break;
    }
}

FsmState_t FSM_GetState(void)
{
    return current_state;
}
