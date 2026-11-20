#ifndef PID_H
#define PID_H

/* 每个 PID_t 实例保存一套参数和独立的控制状态。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_limit;                  // 输出的最大绝对值
    float integral_limit;                // 积分累积值的最大绝对值，0 表示不限幅
    float integral_separation_threshold; // 误差绝对值达到该值时停止累积，0 表示不分离

    float target;
    float feedback;
    float error;
    float previous_error;
    float integral;
    float p_out;
    float i_out;
    float d_out;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit,
              float integral_limit, float integral_separation_threshold);

float PID_Update(PID_t *pid, float target, float feedback);
void PID_Reset(PID_t *pid);

void SetTarget(PID_t *pid, float target);
void SetFeedback(PID_t *pid, float feedback);
void SetK(PID_t *pid, float kp, float ki, float kd);
void SetOutputLimit(PID_t *pid, float output_limit);
void SetIntegralLimit(PID_t *pid, float integral_limit);
void SetIntegralSeparation(PID_t *pid, float threshold);

float getOutput(const PID_t *pid);


#endif /* PID_H */
