#include "fsm.h"
#include "DT7.h"
#include "main.h"

#define UP     1 // 拨杆上档
#define MIDDLE 3 // 拨杆中档
#define DOWN   2 // 拨杆下档

static FsmState_t current_state = FSM_STATE_STOP; // 当前模式，默认停机

/********************************* 模式读取 ********************************************/

/**
 * @brief 读取遥控器两个拨杆的位置，转换为对应的模式事件
 * @return 左中右下进入手动，左下右中进入视觉，其余组合停机
 */
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

/********************************* 主要函数 ********************************************/

/**
 * @brief 根据当前遥控器拨杆位置更新状态机
 */
void FSM_Update(void)
{
    FSM_HandleEvent(read_mode_event());
}

/**
 * @brief 初始化状态机，将当前模式设为停机
 */
void FSM_Init(void)
{
    current_state = FSM_STATE_STOP;
}

/**
 * @brief 根据传入事件切换模式，未知事件保持当前状态
 * @param event 停机、进入手动或进入视觉事件
 */
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

/********************************* 接口函数 ********************************************/

/**
 * @brief 获取状态机当前模式
 * @return 当前的停机、手动或视觉模式
 */
FsmState_t FSM_GetState(void)
{
    return current_state;
}
