#include "PID.h"


//
// @简介：对PID控制器进行初始化
// @参数 Kp - 比例系数
// @参数 Ki - 积分系数
// @参数 Kd - 微分系数
//
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->SP = 0.0f;
	
	PID->err_k_1 = 0.0f;

	
	PID->UpperLimit = +3.4e+38f;
	PID->LowerLimit = -3.4e+38f;
}

//
// @简介：设置PID控制器门限
// @参数 Upper - 上限值
// @参数 Lower - 下限值
//
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower)
{
	PID->UpperLimit = Upper;
	PID->LowerLimit = Lower;
}

//
// @简介：改变设定值SP
// @参数 SP - 新的设定值
//
void PID_ChangeSP(PID_TypeDef *PID, float SP)
{
	PID->SP = SP;
}


//
// @简介：增量式PID
// @参数 FB - 反馈的值，也就是传感器采回的值
// @返回值：PID控制器计算的结果
// FB+=Kp*[e(k)-e(k-1)]+Ki*e(k)+Kd*[e(k)-2e(k-1)+e(k-2)]
// 累加型，value+=PID_Compute_Incremental(PID_TypeDef *PID, float FB)
float PID_Compute_Incremental(PID_TypeDef *PID, float FB)
{
	//计算当前误差
	float err = PID->SP - FB;
	
	//计算比例项
	float Kp_value=PID->Kp*err -PID->Kp*PID->err_k_1;
	//计算积分项目
	float Ki_value=PID->Ki*err;
	//计算微分项目
	float Kd_value=PID->Kd*err -2*PID->Kd*PID->err_k_1+PID->Kd*PID->err_k_2;
	
	//计算PID结果
	float PID_value=Kp_value +Ki_value +Kd_value ;
	
	//保存ek-1与ek-2
  PID->err_k_2=PID->err_k_1;
  PID->err_k_1=err;
	
	return PID_value ;
}

//
// @简介：对PID控制器进行复位
//
void PID_Reset(PID_TypeDef *PID)
{
	PID->err_k_1 = 0.0f;
	PID->err_k_2 = 0.0f;

}

//
// @简介：对位置式PID控制器进行初始化
// @参数 Kp - 比例系数
// @参数 Ki - 积分系数
// @参数 Kd - 微分系数
//
void PID_Pos_Init(PID_Pos_TypeDef *PID_Pos, float Kp, float Ki, float Kd)
{
	PID_Pos->Kp=Kp;
	PID_Pos->Ki=Ki;
	PID_Pos->Kd=Kd;

	PID_Pos->SP=0.0f;
	PID_Pos->err_k_1=0.0f;
	PID_Pos->err_k_1=0.0f;
	PID_Pos->t_k_1=0;

	PID_Pos->UpperLimit = +3.4e+38f;
	PID_Pos->LowerLimit = -3.4e+38f;
}

//
// @简介：改变设定值SP
// @参数 SP - 新的设定值
//
void PID_Pos_ChangeSP(PID_Pos_TypeDef *PID_Pos, float SP)
{
	PID_Pos->SP=SP;
}

//
// @简介：设置PID控制器门限
// @参数 Upper - 上限值
// @参数 Lower - 下限值
//
void PID_Pos_LimitConfig(PID_Pos_TypeDef *PID_Pos, float Upper, float Lower)
{
	PID_Pos->LowerLimit=Lower;
	PID_Pos->UpperLimit=Upper;
}

//
// @简介：对PID控制器进行复位
//
void PID_Pos_Reset(PID_Pos_TypeDef *PID_Pos)
{
	PID_Pos->t_k_1 = 0;
	PID_Pos->err_k_1 = 0.0f;
	PID_Pos->err_int_k_1 = 0.0f;
}

//
// @简介：位置式PID计算
// @参数 FB - 反馈的值，也就是传感器采回的值
// @返回值：PID控制器计算的结果
// Co=Kp*err+Ki*f(err)dt+Kd*derr/dt
// 位置式，value=PID_Compute_Incremental(PID_TypeDef *PID, float FB)
float PID_Compute_Pos(PID_Pos_TypeDef *PID_Pos, float FB)
{

	float err = PID_Pos->SP - FB;

	uint64_t t_k = GetUs();
	float deltaT = (t_k - PID_Pos->t_k_1)* 1.0e-6f;/*这里的单位是s*/

	float err_dev = 0.0f;/*微分值*/
	float err_int = 0.0f;/*积分值*/

	if(PID_Pos->t_k_1 != 0)/*第一次不进行积分与微分*/
	{
		err_dev = (err - PID_Pos->err_k_1) / deltaT;
		err_int = PID_Pos->err_int_k_1 + (err + PID_Pos->err_k_1) * deltaT * 0.5f;
	}

	float COp = PID_Pos->Kp * err;
	float COi = PID_Pos->Ki * err_int;
	float COd = PID_Pos->Kd * err_dev;
	float CO = COp + COi + COd;

	// 更新
	PID_Pos->t_k_1 = t_k;
	PID_Pos->err_k_1 = err;
	PID_Pos->err_int_k_1 = err_int;

	// 输出限幅
	if(CO > PID_Pos->UpperLimit) CO = PID_Pos->UpperLimit;
	if(CO < PID_Pos->LowerLimit) CO = PID_Pos->LowerLimit;

	// 积分限幅
	if(PID_Pos->err_int_k_1 > PID_Pos->UpperLimit) PID_Pos->err_int_k_1 = PID_Pos->UpperLimit;
	if(PID_Pos->err_int_k_1 < PID_Pos->LowerLimit) PID_Pos->err_int_k_1 = PID_Pos->LowerLimit;

	return CO;

}
