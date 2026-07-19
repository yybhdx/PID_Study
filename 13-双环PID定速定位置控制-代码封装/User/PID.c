#include "PID.h"
#include "stm32f10x.h" // Device header

/**
 * @brief  PID 控制算法更新计算函数
 * @param  P 指向 PID 控制器结构体的指针
 * @retval 无
 * @details 
 *         1. 更新历史误差：将本次误差 Error0 存入上次误差 Error1，以用于微分项计算。
 *         2. 计算当前误差：本次误差 = 目标设定值 - 实际反馈值。
 *         3. 积分累加：若积分系数 Ki 不为 0，则进行误差累积；若 Ki 为 0，则积分清零以防止未启用积分时产生异常。
 *         4. PID 输出计算：输出 = 比例项(Kp * 误差) + 积分项(Ki * 累积误差) + 微分项(Kd * 两次误差差值)。
 *         5. 输出限幅处理：若计算结果超出设定的最大/最小限制，则进行硬性截断以保护执行机构。
 */
void Pid_Update(Pid_t *P)
{
    /* 1. 滚动更新误差历史 */
    P->Error1 = P->Error0;             // 保存上次误差

    /* 2. 计算当前周期误差 */
    P->Error0 = P->Target - P->Actual; // 误差 = 目标值 - 反馈值

    /* 3. 积分项累加（包含Ki为零时的保护机制） */
    if (P->Ki != 0) 
    {
        P->ErrorInt += P->Error0;      // 积分误差累加
    }
    else 
    {
        P->ErrorInt = 0;               // 若不使用积分控制，清空积分值以防突变
    }

    /* 4. 位置式 PID 核心计算公式 */
    P->Out = P->Kp * P->Error0 + P->Ki * P->ErrorInt + P->Kd * (P->Error0 - P->Error1);

    /* 5. 输出限幅保护 */
    if (P->Out > P->OutMax)
    {
        P->Out = P->OutMax;            // 限制输出不能超过设定的最大值
    } 
    if (P->Out < P->OutMin)
    {
        P->Out = P->OutMin;            // 限制输出不能低于设定的最小值
    } 
}

