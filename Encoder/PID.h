#ifndef _PID_H
#define _PID_H

#include "ti_msp_dl_config.h"
#include "clock.h"

typedef struct 
{
	float Kp; // 比例系数
	float Ki; // 积分项的系数
	float Kd; // 微分项的系数
	float SP; // 用户的设定值
	
	float err_k_1; // err[k-1]，上次运行PID时的误差
	float err_k_2; // err[k-1]，上次运行PID时的误差
	
	float UpperLimit; // 上限
	float LowerLimit; // 下限
}PID_TypeDef;

typedef struct 
{
	float Kp; // 比例系数
	float Ki; // 积分项的系数
	float Kd; // 微分项的系数
	float SP; // 用户的设定值
	
	uint64_t t_k_1; // t[k-1]，上次运行PID的时间
	float err_k_1; // err[k-1]，上次运行PID时的误差
	float err_int_k_1; // err_int[k-1]，上次运行的积分值

	float UpperLimit; // 上限
	float LowerLimit; // 下限
}PID_Pos_TypeDef;


void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_ChangeSP(PID_TypeDef *PID, float SP);
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower);
void PID_Reset(PID_TypeDef *PID);
float PID_Compute_Incremental(PID_TypeDef *PID, float FB);

void PID_Pos_Init(PID_Pos_TypeDef *PID_Pos, float Kp, float Ki, float Kd);
void PID_Pos_ChangeSP(PID_Pos_TypeDef *PID_Pos, float SP);
void PID_Pos_LimitConfig(PID_Pos_TypeDef *PID_Pos, float Upper, float Lower);
void PID_Pos_Reset(PID_Pos_TypeDef *PID_Pos);
float PID_Compute_Pos(PID_Pos_TypeDef *PID_Pos, float FB);

#endif

