#include "motor.h"

static PID_TypeDef pid_motor_l; // 左电机调速系统的PID控制器
static PID_TypeDef pid_motor_r; // 右电机调速系统的PID控制器
static float PID_KP_r=10.0,PID_KI_r=5.0,PID_KD_r=0.52;
static float PID_KP_l=10.0,PID_KI_l=5.0,PID_KD_l=0.52;
//
// @简介：初始化左右电机调速系统
// @PID输入：转速；输出：PWM占空比
// @pwm_period=1000;限幅+-800.0，duty为80%。
// @Kp=12 ki=12 kd=0；
void App_Motor_Init(void)
{
    PID_Init(&pid_motor_l, PID_KP_l, PID_KI_l, PID_KD_l);
	PID_LimitConfig(&pid_motor_l, +900.0f, -900.0f);
	PID_Init(&pid_motor_r, PID_KP_r, PID_KI_r, PID_KD_r);
	PID_LimitConfig(&pid_motor_r, +900.0f, -900.0f);
}

uint64_t lasttime_ms=0;
float pwmDuty_l=0.0f;
float pwmDuty_r=0.0f;
int16_t pwmcount_l=0;
int16_t pwmcount_r=0;

//
// @简介：电机调速系统的进程函数
//
void App_Motor_Proc(void)
{
    // uint64_t currenttime_ms=mspm0_get_clock_ms_v2();
    // if(currenttime_ms-lasttime_ms<2)
    // { }
    // else 
    // {
        // #1. 通过编码器获取左右电机旋转的角速度
        float omega_l=App_Encoder_GetSpeed_Smoothed_L();
        float omega_r=App_Encoder_GetSpeed_Smoothed_R();

        // #2. 计算PID控制器的输出
        pwmcount_l+=PID_Compute_Incremental(&pid_motor_l, omega_l);
        pwmcount_r+=PID_Compute_Incremental(&pid_motor_r, omega_r);

        // #3. 计算PWM输出
        pwmDuty_l=pwmcount_l/10.0f;
        pwmDuty_r=pwmcount_r/10.0f;

        // #4.限幅比较
        if(pwmDuty_l>(pid_motor_l.UpperLimit/10.0f)) pwmDuty_l=(pid_motor_l.UpperLimit/10.0f);
        if(pwmDuty_l<(pid_motor_l.LowerLimit/10.0f)) pwmDuty_l=(pid_motor_l.LowerLimit/10.0f);
        if(pwmDuty_r>(pid_motor_r.UpperLimit/10.0f)) pwmDuty_r=(pid_motor_r.UpperLimit/10.0f);
        if(pwmDuty_r<(pid_motor_r.LowerLimit/10.0f)) pwmDuty_r=(pid_motor_r.LowerLimit/10.0f);

        // #5.设置电机转动pwm
        App_PWM_Set_L(pwmDuty_l);
        App_PWM_Set_R(pwmDuty_r);
    // }
    // lasttime_ms=currenttime_ms;
}

//
// @简介：用来设置左电机的转速Omega的值
// @参数：Omega - 表示电机的转速，单位是rad/s
//
void App_Motor_SetOmega_L(float Omega)
{
    PID_ChangeSP(&pid_motor_l, Omega);
}
//
// @简介：用来设置右电机的转速Omega的值
// @参数：Omega - 表示电机的转速，单位是rad/s
//
void App_Motor_SetOmega_R(float Omega)
{
    PID_ChangeSP(&pid_motor_r, Omega);
}
//
// @简介：开关电机
// @参数 On：控制电机的开关，0 - 关闭 非零 - 开启
//
void App_Motor_Cmd(uint8_t On)
{
    App_PWM_Cmd(On);
	// 在开关电机的同时需要对PID控制器进行复位
	PID_Reset(&pid_motor_l);
	PID_Reset(&pid_motor_r);
    pwmcount_l=0;
    pwmcount_r=0;
}

float get_pwm_duty_l(void)
{
    return pwmDuty_l;
}

float get_pwm_duty_r(void)
{
    return pwmDuty_r;
}

int16_t get_pwm_count_l(void)
{
    return pwmcount_l;
}

int16_t get_pwm_count_r(void)
{
    return pwmcount_r;
}