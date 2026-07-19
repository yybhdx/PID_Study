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
#include "AD.h"
#include "Pid.h"

/*
 * 倒立摆 串级PID 控制 —— 程序总体说明
 * ==========================================================================
 * 双环串级（外环=位置环，内环=角度环），全部在 TIM1 的 1ms 更新中断里调度：
 *
 *   外环 LocationPid ：反馈 = 小车位置 Location      输出 = 角度偏移量
 *   内环 AngePid     ：反馈 = 摆杆角度 Angle        输出 = 电机 PWM
 *
 *   两环耦合：AngePid.Target = CENTER_ANGLE - LocationPid.Out
 *   含义："位置偏了 → 让摆杆稍微往一侧倾斜 → 小车被带着开回去 → 纠正位置"
 *
 * 控制周期（用计数器对 1ms 中断分频）：
 *   内环每   5ms 跑一次(Count1)  —— 要快，及时把摆杆拉回直立
 *   外环每  50ms 跑一次(Count2)  —— 要比内环慢约10倍，否则两环互相打架
 *
 * 保护：摆杆角度超出 2048±500 时认为已无法救回，强制 RunState=0 停机
 *
 * 角度基准：摆杆竖直时角度传感器 ADC 读数约为 2048，故内环目标恒为 CENTER_ANGLE=2048
 */

#define CENTER_ANGLE 2048 // 中心角度值（摆杆竖直时角度传感器的ADC读数，作为内环目标基准）

#define CENTER_RANGE 500 // 中心角度范围区间(在此范围内可以调控，若超过此区间则不可调控，PID程序自动停止)

#define START_PWM 35     // 启摆时施加给电机的PWM脉冲大小（驱动力强度）
#define START_TIME 100   // 每次施加启摆脉冲的持续时间（单位：毫秒）

uint8_t key_num; // 按键的编号，1为按键1，2为按键2，3为按键3，4为按键4

uint8_t RunState; // 系统运行状态：0=停止，1=启摆检测，21~24=右侧起摆脉冲序列，31~34=左侧起摆脉冲序列，4=PID平衡控制

uint16_t Angle; // 摆杆角度：读取自角度传感器的ADC原始数值（范围0~4095，摆杆垂直向上时读数约在2048附近）

int16_t Speed;	  // 小车速度：编码器在1ms中断内的脉冲计数值（通过M法测量，正负号代表小车运动方向）
int16_t Location; // 小车位置：Speed速度值的累加（即对速度进行积分，得到小车相对于起点的累计位移量）

/* 角度环（内环）PID 参数配置 */
Pid_t AngePid = {
	.Target = CENTER_ANGLE, // 目标值设为中心角度基准2048，使摆杆尽力朝向垂直竖直方向稳定
	.Kp = 0.2,              // 比例系数
	.Ki = 0.01,             // 积分系数
	.Kd = 0.4,              // 微分系数

	.OutMax = 100,          // 最大输出限制（对应电机最大正向PWM）
	.OutMin = -100,         // 最小输出限制（对应电机最大反向PWM）
};

/* 位置环（外环）PID 参数配置 */
Pid_t LocationPid = {
	.Target = 0, // 初始目标位置设在起点0附近
	.Kp = 0.4,   // 比例系数
	.Ki = 0.01,  // 积分系数
	.Kd = 4,     // 微分系数

	.OutMax = 100,  // 最大输出限幅
	.OutMin = -100, // 最小输出限幅
};

int main(void)
{
	/* 模块初始化 */
	OLED_Init();	// 显示屏初始化
	LED_Init();		// 指示灯初始化
	Timer_Init();	// 定时器初始化（配置 TIM1 产生 1ms 的定时中断）
	Key_Init();		// 按键输入初始化
	RP_Init();		// 角度传感器初始化（电位器形式的传感器）
	Motor_Init();	// 电机驱动模块初始化（控制小车移动）
	Encoder_Init(); // 编码器输入初始化（用于测量小车速度和计算位移位置）
	Serial_Init();	// 串口通信初始化
	AD_Init();		// 模数转换器初始化（读取角度传感器模拟信号）

	while (1)
	{
		key_num = Key_GetNum(); // 获取当前被按下的按键编号

		/* 状态切换控制逻辑 */
		if (key_num == 1)
		{
			// 按键1用于启动/停止倒立摆控制
			RunState = !RunState; // 在0（停止）和1（开始启摆）之间翻转状态
		}

		if (key_num == 4)
		{
			// 按键4预留逻辑（当前未启用）
			// RunState = 31;
		}

		if (key_num == 2)
		{
			// 按键2用于将外环目标位置正向偏移（向右微调目标位置）
			LocationPid.Target += 408; // 增加的目标位置值相当于编码器正转一圈的脉冲数
			if (LocationPid.Target > 4080)
			{
				LocationPid.Target = 4080; // 设定向右最大位移边界（最多正转十圈）
			}
		}
		if (key_num == 3)
		{
			// 按键3用于将外环目标位置负向偏移（向左微调目标位置）
			LocationPid.Target -= 408; // 减小的目标位置值相当于编码器反转一圈的脉冲数
			if (LocationPid.Target < -4080)
			{
				LocationPid.Target = -4080; // 设定向左最大位移边界（最多反转十圈）
			}
		}

		/* 指示灯状态指示 */
		if (RunState)
		{
			LED_ON(); // 运行状态下亮灯
		}
		else
		{
			LED_OFF(); // 停止状态下熄灯
		}

		// /* 使用电位器实时在线调节内环角度环PID的Kp, Ki, Kd参数（当前注释，如需手动调参可取消注释） */
		// AngePid.Kp = RP_GetValue(1) / 4095.0 * 1.0;
		// AngePid.Ki = RP_GetValue(2) / 4095.0 * 1.0;
		// AngePid.Kd = RP_GetValue(3) / 4095.0 * 1.0;

		// /* 使用电位器实时在线调节外环位置环PID的Kp, Ki, Kd参数（当前注释） */
		// LocationPid.Kp = RP_GetValue(1) / 4095.0 * 1.0;
		// LocationPid.Ki = RP_GetValue(2) / 4095.0 * 1.0;
		// LocationPid.Kd = RP_GetValue(3) / 4095.0 * 9.0;

		OLED_Printf(42, 0, OLED_6X8, "%02d", RunState); // 显示当前系统所处的运行状态机编号

		/* 在屏上显示内环角度控制器的各项参数与当前状态 */
		OLED_Printf(0, 0, OLED_6X8, "Angle");
		OLED_Printf(0, 12, OLED_6X8, "kp:%05.3f", AngePid.Kp);
		OLED_Printf(0, 20, OLED_6X8, "ki:%05.3f", AngePid.Ki);
		OLED_Printf(0, 28, OLED_6X8, "kd:%05.3f", AngePid.Kd);
		OLED_Printf(0, 40, OLED_6X8, "Tar:%04.0f", AngePid.Target);
		OLED_Printf(0, 48, OLED_6X8, "Act:%04d", Angle);
		OLED_Printf(0, 56, OLED_6X8, "Out:%+04.0f", AngePid.Out);

		/* 在屏上显示外环位置控制器的各项参数与当前状态 */
		OLED_Printf(64, 0, OLED_6X8, "Location");
		OLED_Printf(64, 12, OLED_6X8, "kp:%05.3f", LocationPid.Kp);
		OLED_Printf(64, 20, OLED_6X8, "ki:%05.3f", LocationPid.Ki);
		OLED_Printf(64, 28, OLED_6X8, "kd:%05.3f", LocationPid.Kd);
		OLED_Printf(64, 40, OLED_6X8, "Tar:%+05.0f", LocationPid.Target);
		OLED_Printf(64, 48, OLED_6X8, "Act:%+05d", Location);
		OLED_Printf(64, 56, OLED_6X8, "Out:%+04.0f", LocationPid.Out);

		OLED_Update(); // 将显存缓冲区的数据刷新更新到实际屏幕上显示
	}
}

/* 定时器中断服务函数：处理 TIM1 的溢出/更新中断，固定每 1 毫秒（1ms）被硬件调用执行一次 */
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0 = 0; // 启摆检测分频计数器
	static uint16_t Count1 = 0; // 内环（角度控制）分频计数器，满5次对应5ms控制周期
	static uint16_t Count2 = 0; // 外环（位置控制）分频计数器，满50次对应50ms控制周期
	static uint16_t Count_Time; // 启摆脉冲持续时间计数器（递减计数）

	static uint16_t Angle0; // 当前周期采集到的最新摆杆角度值
	static uint16_t Angle1; // 上一个检测周期采集到的摆杆角度值
	static uint16_t Angle2; // 上上一个检测周期采集到的摆杆角度值

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick(); // 定期扫描按键物理状态（提供消抖时基）

		Angle = AD_GetValue(); // 模数转换读取角度传感器当前的角度值

		Speed = Encoder_Get(); // 读取小车速度（以本次1ms内编码器计数值增量为表征，正负代表方向）

		Location += Speed; // 累加小车位移量得到当前小车相对起点的累计位置

		if (RunState == 0) // 状态0：停机保护状态
		{
			Motor_SetPWM(0); // 彻底关闭电机输出
		}
		else if (RunState == 1) // 状态1：能量积蓄与启摆最高点检测状态
		{
			Count0++;
			if (Count0 >= 40) // 每40毫秒进行一次摆杆轨迹判定（分频降低高频噪声干扰）
			{
				Count0 = 0;
				Angle2 = Angle1; // 滚动记录历史角度序列，Angle2保存旧的上次值
				Angle1 = Angle0; // Angle1保存旧的最新值
				Angle0 = Angle;	 // Angle0更新为最新的角度测量值

				/* 
				 * 判断摆杆是否偏向右侧且此时正好达到了右侧摆动的最大振幅点（最高极点）：
				 * 如果连续三次读数都大于 (CENTER_ANGLE + CENTER_RANGE)，说明整体偏右。
				 * 如果 Angle1（上一次的角度）同时小于 Angle0 且小于 Angle2，说明数值在 Angle1 处探底。
				 * （因为摆杆偏右时，ADC反馈读数会往小的方向减小，所以此处“凹字形”代表了右摆最高点，速度为零，即将朝左摆回）。
				 */
				if (Angle0 > CENTER_ANGLE + CENTER_RANGE && Angle1 > CENTER_ANGLE + CENTER_RANGE && Angle2 > CENTER_ANGLE + CENTER_RANGE)
				{
					if (Angle1 < Angle0 && Angle1 < Angle2) 
					{
						RunState = 21; // 触发右摆起摆驱动序列第一阶段
					}
				}

				/* 
				 * 判断摆杆是否偏向左侧且此时正好达到了左侧摆动的最大振幅点（最高极点）：
				 * 如果连续三次读数都小于 (CENTER_ANGLE - CENTER_RANGE)，说明整体偏左。
				 * 如果 Angle1 同时大于 Angle0 且大于 Angle2，说明数值在 Angle1 处探顶。
				 * （因为摆杆偏左时，ADC反馈读数会往大的方向增加，所以此处“凸字形”代表了左摆最高点，速度为零，即将朝右摆回）。
				 */
				if (Angle0 < CENTER_ANGLE - CENTER_RANGE && Angle1 < CENTER_ANGLE - CENTER_RANGE && Angle2 < CENTER_ANGLE - CENTER_RANGE)
				{
					if (Angle1 > Angle0 && Angle1 > Angle2) 
					{
						RunState = 31; // 触发左摆起摆驱动序列第一阶段
					}
				}
			}
		}
		
		/* 右侧最高点起摆驱动序列（通过小车的左右急移动顺势对摆杆做功，使之摆幅增大） */
		else if (RunState == 21) // 状态21：向左顺势推摆
		{
			Motor_SetPWM(START_PWM); // 电机向左急加速拉车，利用惯性使摆杆向右甩得更高
			Count_Time = START_TIME; // 设定加力时间为100ms
			RunState = 22;			 // 进入第一阶段加力延时等待
		}
		else if (RunState == 22) // 状态22：第一阶段加力延时计数
		{
			Count_Time--;
			if (Count_Time == 0) // 100ms加力结束
			{
				RunState = 23; // 切换到第二阶段
			}
		}
		else if (RunState == 23) // 状态23：向右反向拉摆
		{
			Motor_SetPWM(-START_PWM); // 待摆杆往左回摆时，小车立刻急加速往右拉，顺势增加摆杆回摆动能
			Count_Time = START_TIME;  // 设定加力时间为100ms
			RunState = 24;			  // 进入第二阶段加力延时等待
		}
		else if (RunState == 24) // 状态24：第二阶段加力延时计数
		{
			Count_Time--;
			if (Count_Time == 0) // 100ms反向加力结束
			{
				Motor_SetPWM(0); // 暂时关闭电机输出，让小车滑行，使摆杆自由荡起
				RunState = 1;	 // 重新回到状态1，继续等待下一次更高点判定，实现逐步累积能量直至直立
			}
		}

		/* 左侧最高点起摆驱动序列 */
		else if (RunState == 31) // 状态31：向右顺势推摆
		{
			Motor_SetPWM(-START_PWM); // 电机向右急加速拉车，利用惯性使摆杆向左甩得更高
			Count_Time = START_TIME;  // 设定加力时间为100ms
			RunState = 32;			 // 进入第一阶段加力延时等待
		}
		else if (RunState == 32) // 状态32：第一阶段加力延时计数
		{
			Count_Time--;
			if (Count_Time == 0)
			{
				RunState = 33; // 切换到第二阶段
			}
		}
		else if (RunState == 33) // 状态33：向左反向拉摆
		{
			Motor_SetPWM(START_PWM); // 待摆杆往右回摆时，小车立刻急加速往左拉，顺势增加摆杆回摆动能
			Count_Time = START_TIME; // 设定加力时间为100ms
			RunState = 34;			 // 进入第二阶段加力延时等待
		}
		else if (RunState == 34) // 状态34：第二阶段加力延时计数
		{
			Count_Time--;
			if (Count_Time == 0)
			{
				Motor_SetPWM(0); // 停止加力，切断电机输出
				RunState = 1;	 // 重新回到状态1，继续积累摆幅能量
			}
		}

		/* 闭环平衡控制程序 */
		else if (RunState == 4) // 状态4：双环串级 PID 稳定控制状态
		{
			/* 软件安全防护限幅：
			 * 如果角度读数超出了设定的可控区间（[CENTER_ANGLE - CENTER_RANGE, CENTER_ANGLE + CENTER_RANGE]），
			 * 代表摆杆发生了剧烈晃动或者已经无法挽回地倾倒。
			 * 此时为防止小车发疯撞向轨道端点或损坏电机，必须强制转换状态到0进行软件停机。
			 */
			if (Angle < CENTER_ANGLE - CENTER_RANGE || Angle > CENTER_ANGLE + CENTER_RANGE)
			{
				RunState = 0; // 触发停机
			}
			
			/* 内环角度控制计算（5ms执行周期） */
			Count1++;
			if (Count1 >= 5) 
			{
				Count1 = 0; // 重置计数器

				AngePid.Actual = Angle; // 载入当前的实际摆杆角度反馈

				Pid_Update(&AngePid); // 运行角度环 PID 控制器计算算法

				Motor_SetPWM(AngePid.Out); // 将角度环算出的控制信号直接作为电机的 PWM 输出，以快速稳住摆杆
			}

			/* 外环位置控制计算（50ms执行周期） */
			Count2++;
			if (Count2 >= 50) 
			{
				Count2 = 0; // 重置计数器

				LocationPid.Actual = Location; // 载入当前的实际小车位置反馈

				Pid_Update(&LocationPid); // 运行位置环 PID 控制器计算算法

				/* 串级双环的耦合关系：
				 * 外环（位置环）的输出用来修正内环（角度环）的目标期望角度。
				 * 调整极性逻辑：如果小车发生偏右（反馈位置大于目标位置），则外环输出为正值。
				 * 为了让小车向左回退，我们需要控制小车车体将摆杆向左倾斜（也就是减小内环目标期望值，使得摆杆向左倒），
				 * 小车就会被带回左边。因此，这里应该用减法关系进行串联。
				 */
				AngePid.Target = CENTER_ANGLE - LocationPid.Out; 
			}
		}
	}

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除定时器更新中断标志位，避免重复进入中断
}
