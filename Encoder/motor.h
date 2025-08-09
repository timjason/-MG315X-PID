#ifndef _motor_H
#define _motor_H
#include "ti_msp_dl_config.h"
#include "PID.h"
#include "pwm.h"
#include "encoder.h"
#include "clock.h"
#include "key.h"

void App_Motor_Init(void);
void App_Motor_Proc(void);
void App_Motor_SetOmega_L(float Omega);
void App_Motor_SetOmega_R(float Omega);
void App_Motor_Cmd(uint8_t On);
float get_pwm_duty_l(void);
float get_pwm_duty_r(void);
int16_t get_pwm_count_l(void);
int16_t get_pwm_count_r(void);



#endif