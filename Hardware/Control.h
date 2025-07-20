#ifndef __CONTROL_H
#define __CONTROL_H
 
#include "stm32f10x.h"
 
// 参数宏定义
#define WHEEL_DIAMETER_MM   96.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040
#define PI                  3.1415926535f
 
// 函数声明
// 根据给定的角度和最大速度旋转车辆
void turn_degrees(float angle, int max_speed);
// 根据给定的距离和最大速度直线行驶
void move_straight(float distance_cm, int max_speed);
// 根据给定的速度跟随黑线，直到没有检测到黑线
void follow_line_until_no_line(int speed);
// 发出节点警告信号
void node_alert(void);
// 计算跟踪PID控制器的输出值
int Tracking_PID_Calculate(int error);
// 根据传感器状态计算跟踪误差
int Tracking_Calculate_Error(uint8_t sensor_status);
 
#endif
