#include "task.h"
#include "fsm.h"
#include "DT7.h"
#include "motor.h"
#include "user_can.h"
#include "user_uart.h"
#include <math.h>
#include <stdbool.h>
#include "tool.h"

Target gimbal_target; // 云台目标角度，单位 rad
Output gimbal_out;    // 云台电机控制指令原始值

static Motor_t motor[2]; // motor[0] 为Yaw电机，motor[1] 为Pitch电机

/* TIM1 更新周期为 1 ms；修改定时器周期时需同步修改此值。 */
#define CONTROL_DT_S 0.001f

/* 每个轴以首次反馈角度为中心，正负方向各允许 90°（π/2 rad）。 */
#define TARGET_ANGLE_RANGE_RAD 1.57079632679f

PID_t pid_yaw_angle;      // Yaw角度环
PID_t pid_pitch_angle;    // Pitch角度环
PID_t pid_yaw_velocity;   // Yaw速度环
PID_t pid_pitch_velocity; // Pitch速度环

/********************************* 初始化与反馈 ********************************************/

/**
 * @brief 初始化状态机、电机参数、遥控器和PID，并启动CAN及遥控器接收
 */
void Task_Init(void)
{
    FSM_Init();

    // 两个电机使用相同的反馈换算参数
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

    // 角度环输出目标角速度 rad/s，速度环输出电机控制指令原始值
    PID_Init(&pid_yaw_angle, 100.0f, 3.0f, 0.0f, 33.0, 3.0f, 0.5f);
    PID_Init(&pid_pitch_angle, 0.0f, 0.0f, 0.0f, 33.0, 10.0f, 5.0f);
    PID_Init(&pid_yaw_velocity, 1000.0f, 0.0f, 0.0f, 25000.0f, 5000.0f, 2500.0f);
    PID_Init(&pid_pitch_velocity, 0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 2500.0f);

    CAN_ConfigFilter(); // 配置CAN过滤器，启动CAN接收通知
    // USART1通过DMA空闲接收遥控器数据，启动失败进入错误处理
    if (!Receive_dma_idle(&huart1, remote_rx_buffer, sizeof(remote_rx_buffer)))
    {
        Error_Handler();
    }
}

/**
 * @brief 根据CAN反馈ID更新对应电机的数据
 * @param can_id 接收到的CAN报文ID
 * @param data 电机反馈数据，长度为8字节
 */
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

/**
 * @brief 检查两个电机和遥控器是否在线，超时时间为1000 ms
 * @return 全部在线返回 true，任意设备未收到数据或超时返回 false
 */
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

/********************************* 目标与控制 ********************************************/

/**
 * @brief 根据当前模式更新目标角度，并限制在各电机首次角度的正负90度内
 */
void target(void)
{
    switch (FSM_GetState())
    {
        case FSM_STATE_STOP:
            // 停机时目标跟随当前角度，随后仍受首次角度范围限制
            gimbal_target.target_gimbal_yaw   = getTotalAngleRad(&motor[0]);
            gimbal_target.target_gimbal_pitch = getTotalAngleRad(&motor[1]);
            break;
        case FSM_STATE_MANUAL:
            // 摇杆输入转换为角速度，再乘以1 ms周期累加到目标角度
            gimbal_target.target_gimbal_yaw   += 3.14f * get_left_y() * 0.001f;
            gimbal_target.target_gimbal_pitch += 1.54f * get_right_x() * 0.001f;
            break;
        case FSM_STATE_VISION:
            // 视觉控制暂未接入，目标先跟随当前角度
            gimbal_target.target_gimbal_yaw   = getTotalAngleRad(&motor[0]);
            gimbal_target.target_gimbal_pitch = getTotalAngleRad(&motor[1]);
            break;
        default:
            break;
    }

    // 读取上电后首次反馈的累计角度，作为两个轴各自的限幅中心
    const float initial_yaw = getInitialAngleRad(&motor[0]);
    const float initial_pitch = getInitialAngleRad(&motor[1]);
    gimbal_target.target_gimbal_yaw = clamp(gimbal_target.target_gimbal_yaw,
        initial_yaw - TARGET_ANGLE_RANGE_RAD, initial_yaw + TARGET_ANGLE_RANGE_RAD);
    gimbal_target.target_gimbal_pitch = clamp(gimbal_target.target_gimbal_pitch,
        initial_pitch - TARGET_ANGLE_RANGE_RAD, initial_pitch + TARGET_ANGLE_RANGE_RAD);
}

/**
 * @brief 停机控制，清除四个PID的状态并将电机指令置零
 */
void Stop()
{
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_yaw_angle);
    PID_Reset(&pid_pitch_velocity);
    PID_Reset(&pid_yaw_velocity);
    gimbal_out.out_gimbal_pitch = 0.0f;
    gimbal_out.out_gimbal_yaw = 0.0f;
}

/**
 * @brief 手动控制，角度环和速度环串级计算两个电机的输出
 */
void Manual()
{
    float out_yaw_angle = 0.0f;
    float out_pitch_angle = 0.0f;
    
    // Yaw：角度环输出作为速度环目标，再对最终电机指令限幅
    PID_Update(&pid_yaw_angle, gimbal_target.target_gimbal_yaw, getTotalAngleRad(&motor[0]), CONTROL_DT_S);
    out_yaw_angle = getOutput(&pid_yaw_angle);
    PID_Update(&pid_yaw_velocity, out_yaw_angle, getSpeedRadS(&motor[0]), CONTROL_DT_S);
    gimbal_out.out_gimbal_yaw = clamp(getOutput(&pid_yaw_velocity), -25000.0f, 25000.0f);

    // Pitch：使用同样的串级控制，输出限幅按对应电机指令范围设置
    PID_Update(&pid_pitch_angle, gimbal_target.target_gimbal_pitch, getTotalAngleRad(&motor[1]), CONTROL_DT_S);
    out_pitch_angle = getOutput(&pid_pitch_angle);
    PID_Update(&pid_pitch_velocity, out_pitch_angle, getSpeedRadS(&motor[1]), CONTROL_DT_S);
    gimbal_out.out_gimbal_pitch = clamp(getOutput(&pid_pitch_velocity), -16384.0f, 16384.0f);
}

/**
 * @brief 视觉控制预留接口，目前未更新电机输出
 */
void Vision()
{
    // 空出来后续完善
}

/**
 * @brief 根据状态机当前模式执行停机、手动或视觉控制
 */
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

/********************************* 发送与周期任务 ********************************************/

/**
 * @brief 将Yaw、Pitch输出打包到各自的CAN缓冲区，并通过CAN1提交发送
 */
static void motor_control_logic()
{
    // setCAN按槽位写入两个字节，高字节在前
    setCAN(Can_TX_1.txdata, gimbal_out.out_gimbal_yaw, 1);
    setCAN(Can_TX_2.txdata, gimbal_out.out_gimbal_pitch, 2);
   
    sendCAN(&hcan1, Can_TX_1.txHeader, Can_TX_1.txdata, 0x1FF, 0);
    sendCAN(&hcan1, Can_TX_2.txHeader, Can_TX_2.txdata, 0x2FE, 0);
}

/**
 * @brief 每1 ms调用一次，更新状态、检查在线状态、计算目标和输出并发送指令
 */
void TaskUpdate(void)
{
    FSM_Update();
    if (!is_connected())
    {
        // 任意电机或遥控器掉线，切换到停机状态
        FSM_HandleEvent(FSM_EVENT_STOP);
    }
    target();
    Control();
    motor_control_logic();
}
