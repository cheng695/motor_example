#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* 电机反馈换算参数。具体数值由电机型号决定，由调用者初始化。 */
typedef struct
{
    float reduction_ratio;       // 减速比：电机转速 / 输出轴转速 
    float torque_constant;       // 力矩常数，单位 Nm/A 
    float feedback_current_max;  // CAN 反馈电流对应的最大值 
    float current_max;           // 电机最大电流，单位 A 
    float encoder_resolution;    // 编码器单圈计数 
} MotorParams_t;

/* CAN 反馈帧中直接读取的原始数据。 */
typedef struct
{
    uint16_t encoder;
    int16_t rotor_speed_rpm;
    int16_t feedback_current;
    uint8_t temperature_c;
} MotorRawData_t;

/* 换算到常用物理单位后的电机反馈数据。 */
typedef struct
{
    MotorParams_t params;       // 当前电机的型号参数
    MotorRawData_t raw_data;    // 最近一次收到的原始反馈数据

    float angle_deg;            // 转子单圈角度，单位度 
    float angle_rad;            // 转子单圈角度，单位弧度 
    float total_angle_deg;      // 输出轴累计角度，单位度 
    float total_angle_rad;      // 输出轴累计角度，单位弧度
    float initial_angle_rad;    // 首次有效反馈的输出轴累计角度，初始化后只记录一次
    float speed_rpm;            // 输出轴转速，单位 rpm 
    float speed_rad_s;          // 输出轴角速度，单位 rad/s 
    float current_a;            // 电流，单位 A 
    float torque_nm;            // 估算力矩，单位 Nm 
    uint8_t temperature_c;      // 温度，单位摄氏度 

    uint32_t last_receive_tick; // 最近一次收到电机反馈时的 HAL 系统节拍（毫秒） 
    bool received;              // 是否至少收到过一帧有效反馈 
    bool is_connect;            // 最近一次检查得到的在线状态
} Motor_t;

/* 初始化电机状态并保存该电机的型号参数。 */
void Motor_Init(Motor_t *motor, const MotorParams_t *params);


/* 更新换算后的电机状态。*/
void MotorUpdate(Motor_t *motor, const uint8_t data[8]);
/* 检查并更新电机的在线状态。 */
bool Motor_IsConnected(Motor_t *motor, uint32_t timeout_ms);

/* 将有符号指令写入 8 字节指令帧的 1～4 号槽位。 */
void setCAN(uint8_t tx_data[8], int16_t data, int id);

/* 电机反馈数据读取接口。 */
/* 获取电机转子单圈角度，单位度。*/
float getAngleDeg(const Motor_t *motor);
/* 获取电机转子单圈角度，单位弧度。*/
float getAngleRad(const Motor_t *motor);
/* 获取电机输出轴累计角度，单位度。*/
float getTotalAngleDeg(const Motor_t *motor);
/* 获取电机输出轴累计角度，单位弧度。*/
float getTotalAngleRad(const Motor_t *motor);
/* 获取首次反馈时的累计角度，与 getTotalAngleRad 使用相同坐标，单位弧度。 */
float getInitialAngleRad(const Motor_t *motor);
/* 获取电机输出轴转速，单位 rpm。*/
float getSpeedRpm(const Motor_t *motor);
/* 获取电机输出轴角速度，单位 rad/s。*/
float getSpeedRadS(const Motor_t *motor);
/* 获取电机电流，单位 A。*/
float getCurrentA(const Motor_t *motor);
/* 获取电机估算力矩，单位 Nm。*/
float getTorqueNm(const Motor_t *motor);
/* 获取电机温度，单位摄氏度。*/
uint8_t getTemperatureC(const Motor_t *motor);
/* 获取电机是否连接。*/
bool getIsConnect(const Motor_t *motor);

#endif /* MOTOR_H */
