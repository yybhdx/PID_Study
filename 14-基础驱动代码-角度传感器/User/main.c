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

/*角度传感器测试*/
/*下载此段程序后，OLED屏幕上会显示一些测试内容*/
int main(void)
{
	/*模块初始化*/
	OLED_Init(); // OLED初始化

	AD_Init(); // AD初始化

	while (1)
	{

		OLED_Printf(0, 0, OLED_8X16, "AD: %04d", AD_GetValue()); // 显示AD值

		OLED_Update(); // 更新显示
	}
}
