#include "Control.h"
#include "math.h"
#include "stdlib.h"
#include "Motor.h"
#include "ENCODER.h"
#include "OLED.h"
#include "Delay.h"
#include "LED.h"
#include "Tracking.h" // 添加此行

// 从main.c移入的全局变量
extern int left_current_pulses;
extern int right_current_pulses;


// 使用编码器实现任意角度转弯（带完整PID控制）//已测试
void turn_degrees(float angle, int max_speed) {
    float center_arc_length = (fabs(angle) / 360.0f) * PI * WHEEL_BASE_MM;
    float turns = center_arc_length / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);

    float turn_kp = 1.8f;
    float turn_ki = 0.005f;
    float turn_kd = 2.0f;
    int min_speed = 150;
    float integral = 0;
    float last_error = 0;
    int direction = (angle > 0) ? 1 : -1;

    Clear_Encoder_Count();

    while (1) {
        left_current_pulses = abs(Read_Left_Encoder());
        right_current_pulses = abs(Read_Right_Encoder());
        int average_pulses = (left_current_pulses + right_current_pulses) / 2;
        int error = required_pulses - average_pulses;
        integral += error;
        float derivative = error - last_error;

        if (abs(error) <= 20 && abs(derivative) <= 5) {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        int pid_output = (int)(turn_kp * error + turn_ki * integral + turn_kd * derivative);
        
        if (pid_output > max_speed) {
            pid_output = max_speed;
            integral -= error; 
        }
        if (pid_output < -max_speed) {
            pid_output = -max_speed;
            integral -= error;
        }
        
        if (pid_output > 0 && pid_output < min_speed) {
            pid_output = min_speed;
        } else if (pid_output < 0 && pid_output > -min_speed) {
            pid_output = -min_speed;
        }

        last_error = error;
        MotorA_SetSpeed(direction * pid_output);
        MotorB_SetSpeed(-direction * pid_output);

        OLED_ShowString(2, 1, "Err: ");
        OLED_ShowSignedNum(2, 5, error, 5);
        OLED_ShowString(3, 1, "PID: ");
        OLED_ShowSignedNum(3, 5, pid_output, 5);
        OLED_ShowString(4, 1, "Ang: ");
        OLED_ShowSignedNum(4, 5, (int)angle, 5);
        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}

// 到达顶点时的声光提示
void node_alert(void)
{
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
    Buzzer_Beep(100);
    LED_Blink(500, 2); // 闪烁2次，每次500ms
}

// 使用编码器和PID控制直线行驶指定距离    //已测试(有微小偏差)
void move_straight(float distance_cm, int max_speed)
{
    float distance_mm = distance_cm * 10.0f;
    float turns = distance_mm / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);

    // 直线保持PID
    float straight_kp = 0.2f;
    float straight_ki = 0.005f;
    float straight_kd = 0.04f;
    int min_speed = 100; // 新增最小速度
    float integral = 0;
    float last_error = 0;

    Clear_Encoder_Count();

    while(1)
    {
        left_current_pulses = Read_Left_Encoder();
        right_current_pulses = Read_Right_Encoder();
        
        int average_pulses = (left_current_pulses + right_current_pulses) / 2;

        // 1. 判断是否到达目标距离
        if (average_pulses >= required_pulses) {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        // 2. 计算直线保持PID
        int error = right_current_pulses - left_current_pulses;
        integral += error;
        // 积分限幅
        if (integral > 5000) integral = 5000;
        if (integral < -5000) integral = -5000;
        float derivative = error - last_error;
        last_error = error;

        int adjustment = (int)(straight_kp * error + straight_ki * integral + straight_kd * derivative);

        // 3. 计算左右轮速度
        int left_speed =   (max_speed+min_speed)/2 - adjustment;
        int right_speed = (max_speed+min_speed)/2+ adjustment;

        // 4. 速度限制
        // 速度限制
        if (left_speed > max_speed) left_speed = max_speed;
        if (left_speed < min_speed) left_speed = min_speed; // 确保不低于最小速度
        if (right_speed > max_speed) right_speed = max_speed;
        if (right_speed < min_speed) right_speed = min_speed; // 确保不低于最小速度

        MotorA_SetSpeed(left_speed);
        MotorB_SetSpeed(right_speed);

        // OLED显示调试信息
        OLED_ShowString(2, 1, "Goal:");
        OLED_ShowSignedNum(2, 6, required_pulses, 5);
        OLED_ShowString(3, 1, "Now :");
        OLED_ShowSignedNum(3, 6, average_pulses, 5);
        OLED_ShowString(4, 1, "L/R:");
        OLED_ShowSignedNum(4, 5, left_speed, 4);
        OLED_ShowSignedNum(4, 10, right_speed, 4);

        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}


// 循迹PID控制器计算
int Tracking_PID_Calculate(int error)
{
    static float tracking_kp = 3.0f;
    static float tracking_ki = 0.01f;
    static float tracking_kd = 2.5f;
    static float integral = 0;
    static float last_error = 0;

    integral += error;
    if (integral > 2000) integral = 2000;
    if (integral < -2000) integral = -2000;

    float derivative = error - last_error;
    int adjustment = (int)(tracking_kp * error + tracking_ki * integral + tracking_kd * derivative);
    last_error = error;
    return adjustment;
}

// 根据循迹传感器状态计算偏差值
int Tracking_Calculate_Error(uint8_t sensor_status)
{
    static int last_error = 0;
    int error = 0;

    // 根据4路循迹传感器状态计算偏差
    // S1 S2 S3 S4
    // 0  0  0  0  -> 丢失目标，保持上次偏差
    // 0  0  0  1  -> 偏右，大偏差
    // 0  0  1  0  -> 偏右，小偏差
    // 0  0  1  1  -> 偏右，中偏差 (S3, S4)
    // 0  1  0  0  -> 偏左，小偏差
    // 0  1  0  1  -> 居中 (S2, S4) - 这种情况可能需要特殊处理或调整权重
    // 0  1  1  0  -> 居中 (S2, S3) - 理想状态，偏差为0
    // 0  1  1  1  -> 偏右，中偏差 (S2, S3, S4)
    // 1  0  0  0  -> 偏左，大偏差
    // 1  0  0  1  -> 居中 (S1, S4) - 这种情况可能需要特殊处理或调整权重
    // 1  0  1  0  -> 偏左，中偏差 (S1, S3)
    // 1  0  1  1  -> 居中 (S1, S3, S4)
    // 1  1  0  0  -> 偏左，中偏差 (S1, S2)
    // 1  1  0  1  -> 居中 (S1, S2, S4)
    // 1  1  1  0  -> 居中 (S1, S2, S3)
    // 1  1  1  1  -> 居中 (所有传感器都在黑线上，可能在宽黑线上)

    switch (sensor_status)
    {
        case 0: error = last_error; break; // 所有传感器都检测不到黑线，保持上次偏差
        case 1: error = 30; break;  // S4
        case 2: error = 10; break;  // S3
        case 3: error = 20; break;  // S3, S4
        case 4: error = -10; break; // S2
        case 5: error = 0; break;   // S2, S4 (居中)
        case 6: error = 0; break;   // S2, S3 (居中)
        case 7: error = 10; break;  // S2, S3, S4
        case 8: error = -30; break; // S1
        case 9: error = 0; break;   // S1, S4 (居中)
        case 10: error = -20; break; // S1, S3
        case 11: error = 0; break;   // S1, S3, S4
        case 12: error = -20; break; // S1, S2
        case 13: error = 0; break;   // S1, S2, S4
        case 14: error = 0; break;   // S1, S2, S3
        case 15: error = 0; break;   // 所有传感器都在黑线上
        default: error = last_error; break; // 未知状态，保持上次偏差
    }
    
    last_error = error;
    return error;
}

// 沿黑线循迹直到所有传感器都检测不到黑线
void follow_line_until_no_line(int speed)
{
    // 不需要距离计算，只依赖循迹传感器状态

    while(1)
    {
        // 1. 读取循迹传感器状态并计算偏差
        uint8_t sensor_status = Tracking_ReadStatus();
        int error = Tracking_Calculate_Error(sensor_status);
        
        // 2. 调用PID函数计算调整值
        int adjustment = Tracking_PID_Calculate(error);

        // 3. 计算左右轮速度
        int left_speed = speed - adjustment;
        int right_speed = speed + adjustment;

        // 4. 速度限制
        if (left_speed > speed) left_speed = speed;
        if (left_speed < 0) left_speed = 0; // 循迹时不允许倒退
        if (right_speed > speed) right_speed = speed;
        if (right_speed < 0) right_speed = 0; // 循迹时不允许倒退

        // 5. 设置电机速度
        MotorA_SetSpeed(left_speed);
        MotorB_SetSpeed(right_speed);

        // 6. 退出条件：所有循迹传感器都检测不到黑线
        if (sensor_status == 0x00) { // 0x00表示所有传感器都检测不到黑线
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        // OLED显示调试信息 (可以根据需要调整显示内容)
        OLED_ShowString(2, 1, "Status:");
        OLED_ShowHexNum(2, 9, sensor_status, 2);
        OLED_ShowString(3, 1, "Err: ");
        OLED_ShowSignedNum(3, 6, error, 4);
        OLED_ShowString(4, 1, "L/R: ");
        OLED_ShowSignedNum(4, 6, left_speed, 4);
        OLED_ShowSignedNum(4, 11, right_speed, 4);

        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}
