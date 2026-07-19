#ifndef PID_H
#define PID_H

/**
 * @brief PID 控制器结构体定义
 * @note 包含 PID 控制所需的所有输入、输出、参数、中间误差状态以及输出限幅值
 */
typedef struct
{
    float Target;    /**< 目标设定值 (Setpoint) */
    float Actual;    /**< 实际反馈值 (Feedback Value) */
    float Out;       /**< PID 计算输出值 (Control Output) */

    float Kp;        /**< 比例系数 (Proportional Gain) */
    float Ki;        /**< 积分系数 (Integral Gain) */
    float Kd;        /**< 微分系数 (Derivative Gain) */

    float Error0;    /**< 当前本次误差 (Current Error: Target - Actual) */
    float Error1;    /**< 上次误差 (Previous Error) */
    float ErrorInt;  /**< 误差积分累加值 (Accumulated Integral Error) */

    float OutMax;    /**< 输出最大限制值 (Maximum Output Limit) */
    float OutMin;    /**< 输出最小限制值 (Minimum Output Limit) */
} Pid_t;

/**
 * @brief  PID 控制器输出更新计算
 * @param  P 指向 PID 结构体变量的指针
 * @retval 无
 * @note   该函数内部根据位置式 PID 公式计算输出，并包含积分分离防爆死逻辑和输出限幅处理
 */
void Pid_Update(Pid_t *P);

#endif

