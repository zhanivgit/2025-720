#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "ENCODER.h"
#include "Serial.h"
#include "Control.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // 为了使用atof

// PID FindLinePID; // 定义移至Control.c
extern PID FindLinePID; // 声明外部全局变量
float rho_err;
int Base_Speed = 200; // 基础速度，可调

int main(void)
{
	OLED_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	
	// 初始化循线PID参数，这些值需要后期调试确定
	PID_Init(3, 0.0, 0.1);

	OLED_ShowString(1, 1, "...");
	
	while (1)
	{
		if (Serial_RxFlag == 1)
		{
			if (Serial_RxPacket[0] == 'S')
			{
				Motor_Stop(); // 未检测到线，停车
			}
			else if (Serial_RxPacket[0] == 'E')
			{
				// 从 'E' 后面的字符开始，转换为浮点数
				rho_err = atof(&Serial_RxPacket[1]);
				
				// 显示解析出的偏差值
				OLED_ShowString(3, 1, "Err: ");
				OLED_ShowSignedNum(3, 6, rho_err, 4);

				// PID计算速度调整量
				float Speed_Adjust = Position_PID_FindLine(rho_err);
				
				// 限制最大调整量
				if(Speed_Adjust > Base_Speed) Speed_Adjust = Base_Speed;
				if(Speed_Adjust < -Base_Speed) Speed_Adjust = -Base_Speed;

				// 计算左右轮速度
				int Left_Speed = Base_Speed - Speed_Adjust;
				int Right_Speed = -(Base_Speed + Speed_Adjust);
				
				// 设置电机速度
				MotorA_SetSpeed(Left_Speed);
				MotorB_SetSpeed(-Right_Speed);
				
				OLED_ShowString(4, 1, "L_S:");
				OLED_ShowSignedNum(4, 5, Left_Speed, 3);
				OLED_ShowString(4, 9, "R_S:");
				OLED_ShowSignedNum(4, 13, Right_Speed, 3);
			}
			
			Serial_RxFlag = 0; // 清除标志位
		}
	}
}
