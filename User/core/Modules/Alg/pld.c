#include "pid.h"
#include <math.h>

/********************************* 工具函数 ********************************************/

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static float clamp(float value, float min, float max)
{
    if (value > max)
        return max;

    if (value < -min)
        return -min;

    return value;
}

/********************************* 主要函数 ********************************************/

/**
 * @brief 初始化PID参数
 * @param pid PID参数结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分项系数
 * @param output_limit 输出限幅
 * @param integral_limit 积分限幅
 * @param integral_separation_threshold 积分分离阈值
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit,
              float integral_limit, float integral_separation_threshold)
{
    *pid = (PID_t){0};
    SetK(pid, kp, ki, kd);
    SetOutputLimit(pid, output_limit);
    SetIntegralLimit(pid, integral_limit);
    SetIntegralSeparation(pid, integral_separation_threshold);
}

/**
 * @brief 更新PID参数
 * @param pid PID参数结构体指针
 * @param target 期望值
 * @param feedback 反馈值
 * @return 输出值
 */
float PID_Update(PID_t *pid, float target, float feedback)
{
    SetTarget(pid, target);
    SetFeedback(pid, feedback);

    pid->error = pid->target - pid->feedback;

    // 比例项
    pid->p_out = pid->kp * pid->error;

    // 积分项处理
    // 积分隔离：只有当误差小于阈值时才进行积分累积
    // 当阈值为0时，禁用积分隔离功能，积分始终工作
    if (pid->integral_separation_threshold == 0.0f || fabsf(pid->error) < pid->integral_separation_threshold)
    {
        pid->integral += pid->error;
        if (pid->integral_limit > 0.0f)
        {
            pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
        }
    }

    pid->i_out = pid->ki * pid->integral;

    // 微分项
    pid->d_out = pid->kd * (pid->error - pid->previous_error);

    // 计算输出
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    // 输出限幅
    pid->output = clamp(pid->output, -pid->output_limit, pid->output_limit);

    // 积分抗饱和：如果输出饱和，则回退积分累积
    if ((pid->output >=  pid->output_limit && pid->error > 0.0f) ||
        (pid->output <= -pid->output_limit && pid->error < 0.0f))
    {
        pid->integral -= pid->error;
        // 确保积分不超出限制
        if (pid->integral_limit > 0.0f)
        {
            pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
        } 
    }

    pid->previous_error = pid->error;
    return pid->output;
}

/**
 * @brief 重置PID参数
 * @param pid PID参数结构体指针
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}

/********************************* 接口 函数 ********************************************/
/* 设置期望值 */
void SetTarget(PID_t *pid, float target) { pid->target = target; }
/* 设置反馈值 */
void SetFeedback(PID_t *pid, float feedback) { pid->feedback = feedback; }
/* 设置kp,ki,kd */
void SetK(PID_t *pid, float kp, float ki, float kd) 
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
/* 设置输出限幅 */
void SetOutputLimit(PID_t *pid, float output_limit) { pid->output_limit = fabsf(output_limit); }
/* 设置积分限幅 */
void SetIntegralLimit(PID_t *pid, float integral_limit)
{
    pid->integral_limit = integral_limit > 0.0f ? integral_limit : 0.0f;
    if (pid->integral_limit > 0.0f)
    {
        pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    }
}
/* 设置积分分离阈值 */
void SetIntegralSeparation(PID_t *pid, float threshold) { pid->integral_separation_threshold = fabsf(threshold); }
/* 获取输出 */
float getOutput(const PID_t *pid) { return pid->output; }
