#ifndef PWM_H
#define PWM_H

#include "ti_msp_dl_config.h"

// void App_PWM_Init(void);
// void App_PWM_Cmd(uint8_t on);
void App_PWM_Set_L(float Duty);
void App_PWM_Set_R(float Duty);
void App_PWM_Cmd(uint8_t pwm_on);

#endif