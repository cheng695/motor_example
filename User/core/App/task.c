#include "task.h"
#include "fsm.h"
#include "DT7.h"
#include "motor.h"
#include "user_can.h"
#include "user_uart.h"
#include <math.h>
#include <stdbool.h>

Target gimbal_target;
Output gimbal_out;

static Motor_t motor[2];

PID_t pid_yaw_angle;
PID_t pid_pitch_angle;
PID_t pid_yaw_velocity;
PID_t pid_pitch_velocity;

void Task_Init(void)
{
    FSM_Init();

    const MotorParams_t dji3508_params = {
        .reduction_ratio = 19.0f,
        .torque_constant = 0.3f,
        .feedback_current_max = 16384.0f,
        .current_max = 20.0f,
        .encoder_resolution = 8192.0f,
    };

    for(int i = 0; i < 2; i++)
    {
        Motor_Init(&motor[i], &dji3508_params);
    }
    DT7_Init(20.0f);

    /* 角度环输出 rpm，速度环输出 C620 电流指令原始值。 */
    PID_Init(&pid_yaw_angle, 0.0f, 0.0f, 0.0f, 33.0, 10.0f, 5.0f);
    PID_Init(&pid_pitch_angle, 0.0f, 0.0f, 0.0f, 33.0, 10.0f, 5.0f);
    PID_Init(&pid_yaw_velocity, 0.0f, 0.0f, 0.0f, 25000.0f, 5000.0f, 2500.0f);
    PID_Init(&pid_pitch_velocity, 0.0f, 0.0f, 0.0f, 25000.0f, 5000.0f, 2500.0f);

    CAN_ConfigFilter();
    if (!Receive_dma_idle(&huart1, remote_rx_buffer, sizeof(remote_rx_buffer)))
    {
        Error_Handler();
    }
}

void Task_MotorFeedback(uint32_t can_id, const uint8_t data[8])
{
    switch (can_id)
    {
        case 0x205: // Yaw 电机
            MotorUpdate(&motor[0], data);
            break;
        case 0x206: // Pitch 电机
            MotorUpdate(&motor[1], data);
            break;
        default:
            break;
    }
}

bool is_connected(void)
{
    for(int i = 0; i < 2; i++)
    {
        if(!Motor_IsConnected(&motor[i], 1000))
        {
            return false;
        }
    }

    if(!DT7_IsConnected(1000))
    {
        return false;
    }

    return true;
}

void target(void)
{
    switch (FSM_GetState())
    {
        case FSM_STATE_STOP:
            gimbal_target.target_gimbal_yaw   = getTotalAngleRad(&motor[0]);
            gimbal_target.target_gimbal_pitch = getTotalAngleRad(&motor[1]);
            break;
        case FSM_STATE_MANUAL:
            gimbal_target.target_gimbal_yaw   += 1.57f * get_left_y();
            gimbal_target.target_gimbal_pitch += 0.78f * get_right_x();
            break;
        case FSM_STATE_VISION:
            gimbal_target.target_gimbal_yaw   = getTotalAngleRad(&motor[0]);
            gimbal_target.target_gimbal_pitch = getTotalAngleRad(&motor[1]);
            break;
        default:
            break;
    }
}

void Stop()
{
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_yaw_angle);
    PID_Reset(&pid_pitch_velocity);
    PID_Reset(&pid_yaw_velocity);
    gimbal_out.out_gimbal_pitch = 0.0f;
    gimbal_out.out_gimbal_yaw = 0.0f;
}

void Manual()
{
    float out_yaw_angle = 0.0f;
    float out_pitch_angle = 0.0f;
    
    PID_Update(&pid_yaw_angle, gimbal_target.target_gimbal_yaw, getTotalAngleRad(&motor[0]));
    out_yaw_angle = getOutput(&pid_yaw_angle);
    PID_Update(&pid_yaw_velocity, out_yaw_angle, getSpeedRadS(&motor[0]));
    gimbal_out.out_gimbal_yaw = getOutput(&pid_yaw_velocity);

    PID_Update(&pid_pitch_angle, gimbal_target.target_gimbal_pitch, getTotalAngleRad(&motor[1]));
    out_pitch_angle = getOutput(&pid_pitch_angle);
    PID_Update(&pid_pitch_velocity, out_pitch_angle, getSpeedRadS(&motor[1]));
    gimbal_out.out_gimbal_pitch = getOutput(&pid_pitch_velocity);
}

void Vision()
{
    // 空出来后续完善
}

void Control()
{
    switch (FSM_GetState()) 
    {
        case FSM_STATE_STOP:
            Stop();
            break;
        case FSM_STATE_MANUAL:
            Manual();
            break;
        case FSM_STATE_VISION:
            Vision();
            break;
        default:
            break;
    }
}

void TaskUpdate(void)
{
    FSM_Update();
    if (!is_connected())
    {
        FSM_HandleEvent(FSM_EVENT_STOP);
    }
    target();
    Control();
}
