#include "task.h"
#include "fsm.h"
#include "DT7.h"
#include "motor.h"
#include "user_can.h"
#include "user_uart.h"
#include <math.h>
#include <stdbool.h>
#include "tool.h"

Target gimbal_target;
Output gimbal_out;

static Motor_t motor[2];

/* TIM1 更新周期为 1 ms；修改定时器周期时需同步修改此值。 */
#define CONTROL_DT_S 0.001f

/* 每个轴以首次反馈角度为中心，正负方向各允许 90°（π/2 rad）。 */
#define TARGET_ANGLE_RANGE_RAD 1.57079632679f

PID_t pid_yaw_angle;
PID_t pid_pitch_angle;
PID_t pid_yaw_velocity;
PID_t pid_pitch_velocity;

void Task_Init(void)
{
    FSM_Init();

    const MotorParams_t dji6020_params = {
        .reduction_ratio = 1.0f,
        .torque_constant = 0.741f,
        .feedback_current_max = 16384.0f,
        .current_max = 3.0f,
        .encoder_resolution = 8192.0f,
    };

    for(int i = 0; i < 2; i++)
    {
        Motor_Init(&motor[i], &dji6020_params);
    }
    DT7_Init(20.0f);

    /* 角度环输出 rpm，速度环输出 C620 电流指令原始值。 */
    PID_Init(&pid_yaw_angle, 100.0f, 3.0f, 0.0f, 33.0, 3.0f, 0.5f);
    PID_Init(&pid_pitch_angle, 0.0f, 0.0f, 0.0f, 33.0, 10.0f, 5.0f);
    PID_Init(&pid_yaw_velocity, 1000.0f, 0.0f, 0.0f, 25000.0f, 5000.0f, 2500.0f);
    PID_Init(&pid_pitch_velocity, 0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 2500.0f);

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
        case 0x20A: // Pitch 电机
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
            gimbal_target.target_gimbal_yaw   += 3.14f * get_left_y() * 0.001f;
            gimbal_target.target_gimbal_pitch += 1.54f * get_right_x() * 0.001f;
            break;
        case FSM_STATE_VISION:
            gimbal_target.target_gimbal_yaw   = getTotalAngleRad(&motor[0]);
            gimbal_target.target_gimbal_pitch = getTotalAngleRad(&motor[1]);
            break;
        default:
            break;
    }

    const float initial_yaw = getInitialAngleRad(&motor[0]);
    const float initial_pitch = getInitialAngleRad(&motor[1]);
    gimbal_target.target_gimbal_yaw = clamp(gimbal_target.target_gimbal_yaw,
        initial_yaw - TARGET_ANGLE_RANGE_RAD, initial_yaw + TARGET_ANGLE_RANGE_RAD);
    gimbal_target.target_gimbal_pitch = clamp(gimbal_target.target_gimbal_pitch,
        initial_pitch - TARGET_ANGLE_RANGE_RAD, initial_pitch + TARGET_ANGLE_RANGE_RAD);
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
    
    PID_Update(&pid_yaw_angle, gimbal_target.target_gimbal_yaw, getTotalAngleRad(&motor[0]), CONTROL_DT_S);
    out_yaw_angle = getOutput(&pid_yaw_angle);
    PID_Update(&pid_yaw_velocity, out_yaw_angle, getSpeedRadS(&motor[0]), CONTROL_DT_S);
    gimbal_out.out_gimbal_yaw = clamp(getOutput(&pid_yaw_velocity), -25000.0f, 25000.0f);

    PID_Update(&pid_pitch_angle, gimbal_target.target_gimbal_pitch, getTotalAngleRad(&motor[1]), CONTROL_DT_S);
    out_pitch_angle = getOutput(&pid_pitch_angle);
    PID_Update(&pid_pitch_velocity, out_pitch_angle, getSpeedRadS(&motor[1]), CONTROL_DT_S);
    gimbal_out.out_gimbal_pitch = clamp(getOutput(&pid_pitch_velocity), -16384.0f, 16384.0f);
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

static void motor_control_logic()
{
    setCAN(Can_TX_1.txdata, gimbal_out.out_gimbal_yaw, 1);
    setCAN(Can_TX_2.txdata, gimbal_out.out_gimbal_pitch, 2);
   
    sendCAN(&hcan1, Can_TX_1.txHeader, Can_TX_1.txdata, 0x1FF, 0);
    sendCAN(&hcan1, Can_TX_2.txHeader, Can_TX_2.txdata, 0x2FE, 0);
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
    motor_control_logic();
}
