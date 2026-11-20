#ifndef DT7_H
#define DT7_H

#include <stdbool.h>
#include <stdint.h>

// 通道原始数据（摇杆与开关）
typedef struct
{
    int16_t ch0, ch1, ch2, ch3, scroll;
    uint8_t s1, s2;
} DT7_Channels;

// 坐标数据 [-660,660] (正方向：x前，y右)
typedef struct
{
    int16_t left_stick_x, left_stick_y;
    int16_t right_stick_x, right_stick_y;
    int16_t scroll;
} DT7_Coordinates;

// 摇杆位置 [-1.0,1.0] (正方向：x前，y右)
typedef struct
{
    float left_x, left_y, right_x, right_y, scroll;
} DT7_StickPosition;

// DT7结构体
typedef struct
{
    DT7_Channels channels;
    DT7_Coordinates coordinates;
    DT7_StickPosition stick_position;
    float deadzone;

    // 用于断联判断
    uint32_t last_receive_tick; // 最近一次收到遥控器反馈时的 HAL 系统节拍（毫秒） 
    bool received;              // 是否至少收到过一帧有效反馈 
    bool is_connect;            // 最近一次检查得到的在线状态
} DT7_t;

extern DT7_t remote_DT7;

void DT7_Init(float deadzone);
void DT7_Update(const uint8_t *data);
bool DT7_IsConnected(uint32_t timeout_ms);
/* 使用默认的 100 ms 超时刷新并读取在线状态。 */
bool DT7_GetOnline(void);
/* 设置摇杆死区 */
void DT7_SetDeadzone(float deadzone);
/* 获取原始右摇杆左右数据 */
int16_t get_ch0(void);
/* 获取原始右摇杆前后数据 */
int16_t get_ch1(void);
/* 获取原始左摇杆左右数据 */
int16_t get_ch2(void);
/* 获取原始左摇杆前后数据 */
int16_t get_ch3(void);
/* 获取原始左上滚动轴数据 */
int16_t get_scroll(void);
/* 获取左开关数据 */
int16_t get_s1(void);
/* 获取右开关数据 */
int16_t get_s2(void);
/* 获取归一化后左摇杆前后数据 */
float get_left_x(void);
/* 获取归一化后左摇杆左右数据 */
float get_left_y(void);
/* 获取归一化后右摇杆前后数据 */
float get_right_x(void);
/* 获取归一化后右摇杆左右数据 */
float get_right_y(void);
/* 获取归一化后左上滚动轴数据 */
float get_scroll_(void);
/* 获取遥控器在线状态 */
bool get_online(void);

#endif
