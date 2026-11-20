#include "motor.h"
#include "main.h"
#include <stddef.h>

#define MOTOR_PI 3.14159265358979323846f

/********************************* 工具函数 ********************************************/

/**
 * @brief 计算相邻两次反馈角度的变化量，并处理跨越 0/360 度边界的情况
 * @param current_angle 当前反馈角度，单位为度
 * @param previous_angle 上一次反馈角度，单位为度
 * @return 修正后的角度变化量，范围为 [-180, 180] 度
 */
static float calculate_angle_change(float current_angle, float previous_angle)
{
    float angle_change = current_angle - previous_angle;

    if (angle_change < -180.0f)
        angle_change += 360.0f;
    else if (angle_change > 180.0f)
        angle_change -= 360.0f;

    return angle_change;
}

/**
 * @brief 解析电机反馈数据帧
 * @param motor 电机结构体指针
 * @param params 电机参数结构体指针
 * @param data 反馈数据帧指针
 * @return 解析成功返回 true，否则返回 false
 */
static bool Motor_ReceiveFeedback(Motor_t *motor, const uint8_t data[8])
{
    if (motor == NULL || data == NULL)
        return false;

    motor->raw_data.encoder = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    motor->raw_data.rotor_speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
    motor->raw_data.feedback_current = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    motor->raw_data.temperature_c = data[6];

    motor->last_receive_tick = HAL_GetTick();
    motor->received = true;
    return true;
}

/********************************* 主要函数 ********************************************/

/**
 * @brief 初始化电机结构体
 * @param motor 电机结构体指针
 * @param params 电机参数结构体指针
 */
 void Motor_Init(Motor_t *motor, const MotorParams_t *params)
{
    if (motor == NULL || params == NULL)
        return;

    *motor = (Motor_t){0};
    motor->params = *params;
}

/**
 * @brief 更新电机状态
 * @param motor 电机结构体指针
 */
void MotorUpdate(Motor_t *motor, const uint8_t data[8])
{
    float last_angle;

    if (motor == NULL || data == NULL ||
        motor->params.reduction_ratio <= 0.0f ||
        motor->params.encoder_resolution <= 0.0f ||
        motor->params.feedback_current_max <= 0.0f)
    {
        return;
    }

    const bool first_feedback = !motor->received;
    if (!Motor_ReceiveFeedback(motor, data))
        return;

    last_angle = motor->angle_deg;

    /* 转子角度（度）= 编码器计数 × 360° / 编码器分辨率 */
    motor->angle_deg = (float)motor->raw_data.encoder * 360.0f / motor->params.encoder_resolution;
    /* 转子角度（弧度）= 转子角度（度）× π / 180° */
    motor->angle_rad = motor->angle_deg * MOTOR_PI / 180.0f;
    /* 输出轴转速（rpm）= 转子转速（rpm）/ 减速比 */
    motor->speed_rpm = (float)motor->raw_data.rotor_speed_rpm / motor->params.reduction_ratio;
    /* 输出轴角速度（rad/s）= 输出轴转速（rpm）× 2π / 60 */
    motor->speed_rad_s = motor->speed_rpm * (2.0f * MOTOR_PI / 60.0f);
    /* 输出轴累计角度增量（度）= 修正过零后的转子角度变化量（度）/ 减速比 */
    motor->total_angle_deg += calculate_angle_change(motor->angle_deg, last_angle) /
                              motor->params.reduction_ratio;
    /* 输出轴累计角度增量（弧度）= 输出轴累计角度增量（度）× π / 180° */
    motor->total_angle_rad = motor->total_angle_deg * MOTOR_PI / 180.0f;
    if (first_feedback)
        motor->initial_angle_rad = motor->total_angle_rad;


    /* 实际电流（A）= 反馈电流原始值 × 电机最大电流（A）/ 反馈电流最大值 */
    motor->current_a = (float)motor->raw_data.feedback_current *
                       (motor->params.current_max / motor->params.feedback_current_max);
    /* 估算力矩（Nm）= 实际电流（A）× 力矩常数（Nm/A） */
    motor->torque_nm = motor->current_a * motor->params.torque_constant;
    /* 温度直接取反馈帧中的温度字节，单位为摄氏度 */
    motor->temperature_c = motor->raw_data.temperature_c;
}

/**
 * @brief 检查电机是否连接
 * @param motor 电机结构体指针
 * @param timeout_ms 超时时间（毫秒）
 * @return 连接成功返回 true，否则返回 false
 */
bool Motor_IsConnected(Motor_t *motor, uint32_t timeout_ms)
{
    if (motor == NULL)
        return false;

    if (!motor->received)
    {
        motor->is_connect = false;
    }
    else
    {
        uint32_t elapsed_ms = HAL_GetTick() - motor->last_receive_tick;
        motor->is_connect = elapsed_ms <= timeout_ms;
    }

    return motor->is_connect;
}

/**
 * @brief 设置CAN数据
 * @param tx_data CAN数据指针
 * @param data 数据
 * @param id 电机 ID
 */
void setCAN(uint8_t tx_data[8], int16_t data, int id)
{
    if (tx_data == NULL || id < 1 || id > 4)
    {
        return;
    }

    int offset = (id - 1) * 2;
    uint16_t raw = (uint16_t)data;
    tx_data[offset] = (uint8_t)(raw >> 8);
    tx_data[offset + 1] = (uint8_t)(raw & 0xFFU);
}

/********************************* 接口 函数 ********************************************/
float getAngleDeg(const Motor_t *motor) { return motor->angle_deg; }
float getAngleRad(const Motor_t *motor) { return motor->angle_rad; }
float getTotalAngleDeg(const Motor_t *motor) { return motor->total_angle_deg; }
float getTotalAngleRad(const Motor_t *motor) { return motor->total_angle_rad; }
float getInitialAngleRad(const Motor_t *motor) { return motor->initial_angle_rad; }
float getSpeedRpm(const Motor_t *motor) { return motor->speed_rpm; }
float getSpeedRadS(const Motor_t *motor) { return motor->speed_rad_s; }
float getCurrentA(const Motor_t *motor) { return motor->current_a; }
float getTorqueNm(const Motor_t *motor) { return motor->torque_nm; }
uint8_t getTemperatureC(const Motor_t *motor) { return motor->temperature_c; }
