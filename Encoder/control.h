#ifndef CONTROL_H
#define CONTROL_H

#include "ti_msp_dl_config.h"
#include "PID.h"
#include "mpu6050.h"
#include "clock.h"
#include "encoder.h"
#include "motor.h"
#include "key.h"
#include "bno08x_uart_rvc.h"
#include "follow.h"
#include "PID.h"

void App_Control_Init(void);
float App_Control_Proc(uint8_t cmd,uint8_t run_state , float run_speed , float turn_degree, float pos_set, float Laps);
void App_Control_Reset(void);
void App_Control_SetMoveSpeed(float MoveSpeed);
void App_Control_SetTurnSpeed(float TurnSpeed);
void App_Control_SetYaw_Before(uint8_t cmd);
float Get_BisaDegree(float targe,float current);
extern volatile uint8_t turn_times; 

#endif