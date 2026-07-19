#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "PID.h"

/* ========================================================================= */
/*                                全局变量定义                                */
/* ========================================================================= */

uint8_t KeyNum;       /**< 保存按键键码的变量 */
int16_t Speed;        /**< 实时速度反馈（从编码器读取，带正负号以支持正反转） */
int32_t Location;     /**< 累计位置反馈（对速度进行积分累加得到，带正负号） */

/**
 * @brief 速度内环 PID 结构体初始化
 * @note  输入反馈为 Speed，输出为占空比控制信号，直接输出给电机驱动模块
 */
Pid_t Inner = {
	.Kp = 0.3,
	.Ki = 0.3,
	.Kd = 0,
	.OutMax = 100,  /**< 输出限幅最大值：对应 PWM 占空比 100% */
	.OutMin = -100, /**< 输出限幅最小值：对应逆向 PWM 占空比 100% */
};

/**
 * @brief 位置外环 PID 结构体初始化
 * @note  输入反馈为 Location，输出为目标转速信号，输出给速度内环作为设定值
 */
Pid_t Outer = {
	.Kp = 0.3,
	.Ki = 0,
	.Kd = 0.4,
	.OutMax = 20,   /**< 输出限幅最大值：限制内环目标速度在正向 20 以内 */
	.OutMin = -20,  /**< 输出限幅最小值：限制内环目标速度在反向 20 以内 */
};

/* ========================================================================= */
/*                                  主程序                                   */
/* ========================================================================= */

int main(void)
{
	/* 1. 硬件外设与系统模块初始化 */
	OLED_Init();	// 初始化 OLED 屏幕驱动
	Key_Init();		// 初始化按键扫描驱动（非阻塞式）
	Motor_Init();	// 初始化直流电机及方向引脚驱动
	Encoder_Init(); // 初始化编码器定时器测速接口
	RP_Init();		// 初始化电位器模数转换（ADC）驱动
	Serial_Init();	// 初始化串口通信，波特率配置为 9600

	Timer_Init();   // 初始化定时器 TIM1 中断，配置为每 1ms 产生一次中断

	/* 2. OLED 屏幕初始静态显示 */
	OLED_Printf(0, 0, OLED_8X16, "2*PID Control");
	OLED_Update();  // 更新 OLED 显存到屏幕

	while (1)
	{
		/* 按键控制代码区 (如需使用请取消注释，并屏蔽电位器控制部分) */
		//		KeyNum = Key_GetNum();		// 获取按键键码
		//		if (KeyNum == 1)			// 按键 K1 被按下
		//		{
		//			Inner.Target += 10;		// 目标值累加 10
		//		}
		//		if (KeyNum == 2)			// 按键 K2 被按下
		//		{
		//			Inner.Target -= 10;		// 目标值递减 10
		//		}
		//		if (KeyNum == 3)			// 按键 K3 被按下
		//		{
		//			Inner.Target = 0;		// 目标值复位清零
		//		}

		/* 电位器调参控制区 (此处主要调整位置外环目标位置) */
		/* RP_GetValue(通道) 返回 0~4095 的原始 ADC 采样值 */
		/* 注：由于双环控制中外环定时计算每 40ms 会覆盖内环的目标值 Inner.Target = Outer.Out， */
		/* 因此这里只修改外环目标位置 Outer.Target。如果要单独调试速度内环，应在此给 Inner.Target 赋值。 */
		// Outer.Kp = RP_GetValue(1) / 4095.0 * 2;			   // 电位器1修改外环Kp (范围 0~2)
		// Outer.Ki = RP_GetValue(2) / 4095.0 * 2;			   // 电位器2修改外环Ki (范围 0~2)
		// Outer.Kd = RP_GetValue(3) / 4095.0 * 2;			   // 电位器3修改外环Kd (范围 0~2)
		Outer.Target = RP_GetValue(4) / 4095.0 * 816 - 408; // 电位器4修改外环目标位置 (范围 -408 ~ 408 脉冲，约一圈)

		/* 3. OLED 屏幕信息实时刷新显示 */
		OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Outer.Kp); // 显示当前外环比例系数
		OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Outer.Ki); // 显示当前外环积分系数
		OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Outer.Kd); // 显示当前外环微分系数

		OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Outer.Target); // 显示目标位置
		OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Outer.Actual); // 显示当前实际位置
		OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Outer.Out);    // 显示外环计算输出值（目标速度值）

		OLED_Update(); // 更新 OLED 显存到屏幕显示

		/* 4. 串口调试输出（供 SerialPlot 等上位机软件波形分析） */
		Serial_Printf("%f,%f,%f\r\n", Outer.Target, Outer.Actual, Outer.Out);
	}
}

/* ========================================================================= */
/*                              定时器中断服务程序                           */
/* ========================================================================= */

/**
 * @brief  TIM1 溢出/更新中断服务程序
 * @note   该中断每 1ms 触发一次，用于按键扫描和分频控制双环 PID 周期计算
 */
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count1, Count2; // 声明静态计次分频变量（退出函数后不销毁）

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/* === 1. 毫秒级任务触发 === */
		Key_Tick(); // 按键扫描心跳，每 1ms 调用一次（用于防抖和多击检测）

		/* === 2. 速度内环控制任务 (分频到 40ms 周期执行) === */
		Count1++;
		if (Count1 >= 40) // 每积攒到 40ms 执行一次
		{
			Count1 = 0; // 计数器复位

			/* 2.1 获取电机的当前实时转速（编码器计次增量） */
			Speed = Encoder_Get();
			
			/* 2.2 累加编码器输出以计算电机当前所处的位置 */
			Location += Speed; // 对速度增量进行积分，求出位置积分值

			/* 2.3 执行速度内环计算 */
			Inner.Actual = Speed;    // 内环的反馈源是当前电机速度
			Pid_Update(&Inner);      // 更新速度环输出

			/* 2.4 执行底层硬件控制 */
			Motor_SetPWM(Inner.Out); // 将速度内环计算得出的 PWM 控制信号输出给电机
		}

		/* === 3. 位置外环控制任务 (分频到 40ms 周期执行) === */
		Count2++;
		if (Count2 >= 40) // 每积攒到 40ms 执行一次
		{
			Count2 = 0; // 计数器复位

			/* 3.1 执行位置外环计算 */
			Outer.Actual = Location; // 外环的反馈源是当前累加位置
			Pid_Update(&Outer);      // 更新位置环输出

			/* 3.2 双环级联串联 */
			Inner.Target = Outer.Out; // 将位置环计算输出的目标速度直接赋给速度内环作为目标设定值
		}

		/* 4. 清除定时器中断标志位 */
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

