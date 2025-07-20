#include "Control.h"
#include "Motor.h"
#include "stm32f10x.h"

// 定义一个全局的PID变量
PID FindLinePID;

/**
  * @brief  初始化PID结构体
  * @param  Kp, Ki, Kd: PID参数
  * @retval 无
  */
void PID_Init(float Kp, float Ki, float Kd)
{
    FindLinePID.Kp = Kp;
    FindLinePID.Ki = Ki;
    FindLinePID.Kd = Kd;
    FindLinePID.Error = 0;
    FindLinePID.Last_Error = 0;
    FindLinePID.Integral = 0;
    FindLinePID.Output = 0;
}

/**
  * @brief  位置式PID控制器，用于循线
  * @param  target_error: 目标误差，即OpenMV发来的rho_err
  * @retval 计算出的PID输出值（用于调整电机速度）
  */
float Position_PID_FindLine(float target_error)
{
    FindLinePID.Error = target_error;
    
    // 积分项
    FindLinePID.Integral += FindLinePID.Error;
    
    // 积分限幅，防止积分饱和
    if (FindLinePID.Integral > 3000) FindLinePID.Integral = 3000;
    if (FindLinePID.Integral < -3000) FindLinePID.Integral = -3000;
    
    // PID计算
    FindLinePID.Output = FindLinePID.Kp * FindLinePID.Error + 
                         FindLinePID.Ki * FindLinePID.Integral + 
                         FindLinePID.Kd * (FindLinePID.Error - FindLinePID.Last_Error);
                  
    FindLinePID.Last_Error = FindLinePID.Error;
    
    return FindLinePID.Output;
}
