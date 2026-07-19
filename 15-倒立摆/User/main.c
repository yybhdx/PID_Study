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

uint8_t key_num; // key_num表示按键的编号，1为按键1，2为按键2，3为按键3

uint8_t RunState; // RunState为运行状态：0=停止，1=运行（按键1切换；摆杆倾角超出±500时会被强制清0保护）

uint16_t Angle; // Angle为摆杆角度＝角度传感器ADC原值（0~4095，竖直时约2048）

int16_t Speed;	  // Speed为小车速度＝编码器每1ms的脉冲计数值（单位时间位移；正负代表方向）
int16_t Location; // Location为小车位置＝Speed的累加（积分），即编码器脉冲的累计值

/*写入自己调控得到的内环PID参数*/
Pid_t AngePid = {
	.Target = CENTER_ANGLE, // 为了使倒立摆稳定倒立，目标角度值一定要是中心值2048
	.Kp = 0.2,
	.Ki = 0.01,
	.Kd = 0.4,

	.OutMax = 100,
	.OutMin = -100,
};

// Pid_t AngePid = {
// 	.Target = CENTER_ANGLE, // 为了使倒立摆稳定倒立，目标角度值一定要是中心值2048
// 	.Kp = 0,
// 	.Ki = 0,
// 	.Kd = 0,

// 	.OutMax = 100,
// 	.OutMin = -100,
// };

/*写入自己调控得到的外环PID参数*/
Pid_t LocationPid = {
	.Target = 0, // 初始目标位置在0附近
	.Kp = 0.4,
	.Ki = 0.01,
	.Kd = 4,

	.OutMax = 100,
	.OutMin = -100,
};

// Pid_t LocationPid = {
// 	.Target = 0, // 初始目标位置在0附近
// 	.Kp = 0,
// 	.Ki = 0,
// 	.Kd = 0,

// 	.OutMax = 100,
// 	.OutMin = -100,
// };

/*角度传感器测试*/
/*下载此段程序后，OLED屏幕上会显示一些测试内容*/
int main(void)
{
	/*模块初始化*/
	OLED_Init();	// OLED初始化
	LED_Init();		// LED初始化
	Timer_Init();	// 定时器初始化
	Key_Init();		// 按键初始化
	RP_Init();		// 角度传感器初始化
	Motor_Init();	// 电机初始化
	Encoder_Init(); // 编码器初始化
	Serial_Init();	// 串口初始化
	AD_Init();		// AD初始化

	while (1)
	{
		key_num = Key_GetNum();

		/*切换倒立摆的运行状态*/
		if (key_num == 1)
		{
			// 处理按键1的逻辑
			RunState = !RunState; // 切换运行状态
		}
		if (key_num == 2)
		{
			LocationPid.Target += 408; // 正转一圈
			if (LocationPid.Target > 4080)
			{
				LocationPid.Target = 4080; // 最多正转十圈
			}
		}
		if (key_num == 3)
		{
			LocationPid.Target -= 408; // 反转一圈
			if (LocationPid.Target < -4080)
			{
				LocationPid.Target = -4080; // 最多反转十圈
			}
		}

		/*用LED显示运行状态*/
		if (RunState)
		{
			LED_ON();
		}
		else
		{
			LED_OFF();
		}

		// /*使用电位器调节内环角度环PID的值以及目标值*/
		// AngePid.Kp = RP_GetValue(1) / 4095.0 * 1.0;
		// AngePid.Ki = RP_GetValue(2) / 4095.0 * 1.0;
		// AngePid.Kd = RP_GetValue(3) / 4095.0 * 1.0;

		// /*使用电位器调节外环位置环PID的值以及目标值*/
		// LocationPid.Kp = RP_GetValue(1) / 4095.0 * 1.0;
		// LocationPid.Ki = RP_GetValue(2) / 4095.0 * 1.0;
		// LocationPid.Kd = RP_GetValue(3) / 4095.0 * 9.0;

		/*OLED显示内环角度环的参数*/
		OLED_Printf(0, 0, OLED_6X8, "Angle:%04d", Angle);
		OLED_Printf(0, 12, OLED_6X8, "kp:%05.3f", AngePid.Kp);
		OLED_Printf(0, 20, OLED_6X8, "ki:%05.3f", AngePid.Ki);
		OLED_Printf(0, 28, OLED_6X8, "kd:%05.3f", AngePid.Kd);
		OLED_Printf(0, 40, OLED_6X8, "Tar:%04.0f", AngePid.Target);
		OLED_Printf(0, 48, OLED_6X8, "Act:%04d", Angle);
		OLED_Printf(0, 56, OLED_6X8, "Out:%+04.0f", AngePid.Out);

		/*OLED显示外环位置环的参数*/
		OLED_Printf(64, 0, OLED_6X8, "Location:%04d", Location);
		OLED_Printf(64, 12, OLED_6X8, "kp:%05.3f", LocationPid.Kp);
		OLED_Printf(64, 20, OLED_6X8, "ki:%05.3f", LocationPid.Ki);
		OLED_Printf(64, 28, OLED_6X8, "kd:%05.3f", LocationPid.Kd);
		OLED_Printf(64, 40, OLED_6X8, "Tar:%+05.0f", LocationPid.Target);
		OLED_Printf(64, 48, OLED_6X8, "Act:%+05d", Location);
		OLED_Printf(64, 56, OLED_6X8, "Out:%+04.0f", LocationPid.Out);

		/*将OLED显存数组更新到OLED屏幕*/
		OLED_Update();
	}
}

/*定时器中断函数，用于处理TIM1更新中断，1ms执行一次*/
void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count1 = 0; // 内环(角度环)计数器：计满5次→每5ms执行一次内环（内环要快，及时拉回直立）
	static uint16_t Count2 = 0; // 外环(位置环)计数器：计满50次→每50ms执行一次外环（必须比内环慢约10倍，否则两环打架）
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick(); // 按键扫描时基（每1ms调用一次，配合Key_GetNum做软件消抖）

		Angle = AD_GetValue(); // 读取角度传感器的角度值

		Speed = Encoder_Get(); // 读取编码器速度：返回自上次调用以来的脉冲增量(1ms内)，即速度(M法测速)

		Location += Speed; // 位置＝速度的累加(积分)，得到小车累计位移

		// 保护：摆杆倾角过大(超出2048±500)，已无法救回，强制停机以免失控/撞限位
		if (Angle < CENTER_ANGLE - CENTER_RANGE || Angle > CENTER_ANGLE + CENTER_RANGE)
		{
			// 处理在中心范围外的逻辑
			RunState = 0;
		}

		/*若RunState不为0，则启动PID程序*/
		if (RunState)
		{
			// 处理运行状态的逻辑
			Count1++;
			if (Count1 >= 5) // 内环控制周期=5ms（1ms中断×5）；周期越短响应越快，但占用CPU越多
			{
				// 处理内环调控逻辑
				Count1 = 0; // 重置计数器

				AngePid.Actual = Angle; // 更新角度实际值

				Pid_Update(&AngePid); // PID 控制算法更新计算函数

				Motor_SetPWM(AngePid.Out); // 设置电机PWM值
			}

			Count2++;
			if (Count2 >= 50) // 外环控制周期=50ms（1ms×50），必须慢于内环；太慢位置回正迟，太快则与内环耦合振荡
			{
				// 处理外环调控逻辑
				Count2 = 0; // 重置计数器

				LocationPid.Actual = Location; // 更新位置实际值

				Pid_Update(&LocationPid); // PID 控制算法更新计算函数

				// 若位置环的目标值为0，那么角度环的目标值就是正常的CENTER_ANGLE
				// 若位置环的目标值不为0，那么位置环就会在中心角度的基础上加减，进而调节摆杆左右倾斜，最终控制位置左右移动
				// AngePid.Target = CENTER_ANGLE + LocationPid.Out; // 将位置环外环的输出值作用于内环角度环的目标值

				/*调整极性*/
				AngePid.Target = CENTER_ANGLE - LocationPid.Out; // 将位置环外环的输出值作用于内环角度环的目标值
			}
		}
		else // 不执行PID程序
		{
			Motor_SetPWM(0); // 停止电机
		}

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
