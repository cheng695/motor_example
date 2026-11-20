#include "DT7.h"
#include "main.h"
#include <stddef.h>
#include <stdint.h>

#define DT7_CHANNEL_MID 1024   // 原始中值
#define DT7_CHANNEL_RANGE 660  // 减去中值后的范围
#define DT7_ONLINE_TIMEOUT_MS 100U
#define DT7_ONLINE_TIMEOUT_MS 100U

DT7_t remote_DT7;

/********************************* 工具函数 ********************************************/

/**
 * @brief 对输入值进行死区处理
 * @param value 输入值，通常是摇杆相对中值的偏移量
 * @param deadzone 死区范围；当 value 的绝对值小于该值时，结果置为 0
 * @return 处理后的值；在死区内返回 0，否则返回原值
 */
static int16_t compensate_deadzone(int16_t value, float deadzone)
{
    return ((value < 0 ? -(int32_t)value : (int32_t)value) < deadzone) ? 0 : value;

    /* 上述函数展开等价于：
    int32_t absolute_value;

    if (value < 0)
    {
        absolute_value = -(int32_t)value;
    }
    else
    {
        absolute_value = value;
    }

    if (absolute_value < deadzone)
    {
        return 0;
    }

    return value;
    */
}

/**
 * @brief 将[-660,660]映射至[-1,1]
 * @param value 输入值，通常是摇杆相对中值的偏移量
 * @return 处理后的值；在死区内返回 0，否则返回原值
 */
static float discrete_axis(int16_t value)
{
    float normalized = (float)value / (float)DT7_CHANNEL_RANGE;
    if (normalized > 1.0f) return 1.0f;
    if (normalized < -1.0f) return -1.0f;
    return normalized;
}

/********************************* 主要函数 ********************************************/

/**
 * @brief 初始化
 * @param deadzone 摇杆死区范围；当摇杆的绝对值小于该值时，结果置为 0
 */
void DT7_Init(float deadzone)
{
    remote_DT7.deadzone = 0.0f;
    remote_DT7.received = false;
    remote_DT7.is_connect = false;
    DT7_SetDeadzone(deadzone);
}

/**
 * @brief 解析数据
 * @param data 输入数据
 */
void DT7_Update(const uint8_t *data)
{
    if (data == NULL)
        return;

    remote_DT7.last_receive_tick = HAL_GetTick();
    remote_DT7.received = true;

    // 原始数据
    remote_DT7.channels.ch0 = (int16_t)(((uint16_t)data[0] | ((uint16_t)data[1] << 8)) & 0x07FFU);
    remote_DT7.channels.ch1 = (int16_t)((((uint16_t)data[1] >> 3) | ((uint16_t)data[2] << 5)) & 0x07FFU);
    remote_DT7.channels.ch2 = (int16_t)((((uint16_t)data[2] >> 6) | ((uint16_t)data[3] << 2) | ((uint16_t)data[4] << 10)) & 0x07FFU);
    remote_DT7.channels.ch3 = (int16_t)((((uint16_t)data[4] >> 1) | ((uint16_t)data[5] << 7)) & 0x07FFU);
    remote_DT7.channels.s1  = (uint8_t)((data[5] >> 4) & 0x03U);
    remote_DT7.channels.s2  = (uint8_t)((data[5] >> 6) & 0x03U);
    /* 中间的键鼠省略 */
    remote_DT7.channels.scroll = (int16_t)((uint16_t)data[16] | ((uint16_t)data[17] << 8));

    // 映射至[-660,660]，考虑死区
    remote_DT7.coordinates.left_stick_x  = compensate_deadzone((int16_t)(remote_DT7.channels.ch3 - DT7_CHANNEL_MID), remote_DT7.deadzone);
    remote_DT7.coordinates.left_stick_y  = compensate_deadzone((int16_t)(remote_DT7.channels.ch2 - DT7_CHANNEL_MID), remote_DT7.deadzone);
    remote_DT7.coordinates.right_stick_x = compensate_deadzone((int16_t)(remote_DT7.channels.ch1 - DT7_CHANNEL_MID), remote_DT7.deadzone);
    remote_DT7.coordinates.right_stick_y = compensate_deadzone((int16_t)(remote_DT7.channels.ch0 - DT7_CHANNEL_MID), remote_DT7.deadzone);
    remote_DT7.coordinates.scroll = compensate_deadzone((int16_t)(remote_DT7.channels.scroll - DT7_CHANNEL_MID), remote_DT7.deadzone);

    // 映射至[-1,1]
    remote_DT7.stick_position.left_x  = discrete_axis(remote_DT7.coordinates.left_stick_x);
    remote_DT7.stick_position.left_y  = discrete_axis(remote_DT7.coordinates.left_stick_y);
    remote_DT7.stick_position.right_x = discrete_axis(remote_DT7.coordinates.right_stick_x);
    remote_DT7.stick_position.right_y = discrete_axis(remote_DT7.coordinates.right_stick_y);
    remote_DT7.stick_position.scroll  = discrete_axis(remote_DT7.coordinates.scroll);
}

/**
 * @brief 检查遥控器是否在线
 * @return 1 表示在线；0 表示尚未收到数据或已断联
 */
bool DT7_IsConnected(uint32_t timeout_ms)
{
    if (!remote_DT7.received)
    {
        remote_DT7.is_connect = false;
    }
    else
    {
        uint32_t elapsed_ms = HAL_GetTick() - remote_DT7.last_receive_tick;
        remote_DT7.is_connect = elapsed_ms <= timeout_ms;
    }

    return remote_DT7.is_connect;
}

/********************************* 接口 函数 ********************************************/
void DT7_SetDeadzone(float deadzone)
{
    remote_DT7.deadzone = deadzone < 0.0f ? 0.0f : deadzone;
}

// 通道数据(中值1024)
int16_t get_ch0(void) { return remote_DT7.channels.ch0; }
int16_t get_ch1(void) { return remote_DT7.channels.ch1; }
int16_t get_ch2(void) { return remote_DT7.channels.ch2; }
int16_t get_ch3(void) { return remote_DT7.channels.ch3; }
int16_t get_scroll(void) { return remote_DT7.channels.scroll; }

// 开关数据
int16_t get_s1(void) { return remote_DT7.channels.s1; }
int16_t get_s2(void) { return remote_DT7.channels.s2; }

// 摇杆数据(-1--1)
float get_left_x(void) { return remote_DT7.stick_position.left_x; }
float get_left_y(void) { return remote_DT7.stick_position.left_y; }
float get_right_x(void) { return remote_DT7.stick_position.right_x; }
float get_right_y(void) { return remote_DT7.stick_position.right_y; }
float get_scroll_(void) { return remote_DT7.stick_position.scroll; }

// 获取在线状态
bool DT7_GetOnline(void) { return DT7_IsConnected(DT7_ONLINE_TIMEOUT_MS); }
