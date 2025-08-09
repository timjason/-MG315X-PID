#include "control.h"

float Speed_Kp,Speed_Ki,Speed_Kd;
float Turn_Kp=3.0,Turn_Ki=0.27,Turn_Kd=0.106;
float Yaw_berfore;
float Yaw_now;
// float Turn_Kp=20.0,Turn_Ki=5.5,Turn_Kd=0.0;
static PID_TypeDef pid_turn={0}; // 转向环
static PID_Pos_TypeDef pid_follow={0};//循迹控制环
float Get_BisaDegree(float targe,float current);

//
// @简介：初始化转向环
//
void App_Control_Init(void)
{
    PID_Init(&pid_turn, Turn_Kp, Turn_Ki, Turn_Kd);
    PID_LimitConfig(&pid_turn, 0.5, -0.5);
    PID_Pos_Init(&pid_follow, 0.6, 0.8,0.3);
    PID_Pos_LimitConfig(&pid_follow, 1.5, -1.5);
}

float speed_set=0.0f; //速度设定值
float turn_omega_set=0.0f; //转向设定值
static uint64_t lasttime=0; //上次运行control_pid时间
float speed=0.0;
static uint8_t a_Flag=0;//加速度是否完成；0-没完成，1-已经完成
static float Speed_setnow=0.0f;//当前设置的速度
static uint8_t a_thrid_Flag=0;//首3次缓慢加速
static uint8_t turn_cal_flag=0;//是否计算了目标角度的标志
static float targe_degree=0.0;//目标角度
static float omega_bais=0.0;//设定角与当前角度误差
static uint8_t set_flag=0;//是否设定循迹环目标位置
static uint8_t slow_down_flag=0;//减速标志位
static uint16_t encoder_count=0;//编码器计数
volatile uint8_t turn_times=0;//转弯次数
volatile uint8_t lastturn_state=0;//上次转弯状态
uint8_t turn_now=0;
//
// @简介：小车控制系统的进程函数，在while循环中调用
// @输入：cmd 是否执行整个函数；0-执行，1-执行
//        run_state 运动类型编号0-静止，1-按设定速度直行，2-按设定速度转(左-yaw；右+yaw)，3-循迹
//        run_speed 运动速度，单位：rad/s
//        turn_degree 转向角度，单位：度；当run_state=0，1时忽略这个参数
//        pos_set 循迹相关的设定位置
//        Laps 圈数
float App_Control_Proc(uint8_t cmd,uint8_t run_state , float run_speed , float turn_degree, float pos_set, float Laps)
{
    uint64_t currenttime=mspm0_get_clock_ms_v2();
    float Omega=0.0f;
    if(currenttime-lasttime<5)//5ms更新一次
    {

    }
    else if(cmd!=0)
    {
        switch (run_state) 
        {
            case 0:/*静止*/
            App_Control_SetMoveSpeed(0.0);
            App_Motor_SetOmega_L(0.0);
            App_Motor_SetOmega_R(0.0);
            break;

            case 1:/*按照设定速度直行*/
            Yaw_now=bno08x_data.yaw;/*当前yaw角*/
            App_Control_SetTurnSpeed(Yaw_berfore);
            App_Control_SetMoveSpeed(run_speed);

            if (a_Flag==0) {
                if (a_thrid_Flag<3) 
                {
                    Speed_setnow+=0.3;
                }
                else {
                    Speed_setnow+=0.5;
                }
                if(Speed_setnow>=speed_set) {
                    Speed_setnow=speed_set;
                    a_Flag=1;
                }
            }
            PID_LimitConfig(&pid_turn, +Speed_setnow/20.0f,-Speed_setnow/20.0f);
            turn_omega_set+=PID_Compute_Incremental(&pid_turn, Yaw_now);
            if (turn_omega_set>pid_turn.UpperLimit) turn_omega_set=pid_turn.UpperLimit;
            if (turn_omega_set<pid_turn.LowerLimit) turn_omega_set=pid_turn.LowerLimit;
            App_Motor_SetOmega_L(Speed_setnow+(turn_omega_set*Speed_setnow/speed_set));
            App_Motor_SetOmega_R(Speed_setnow-(turn_omega_set*Speed_setnow/speed_set));
            break;

            case 2:/*按设定速度转(左-yaw；右+yaw)*/
            Yaw_now=bno08x_data.yaw;/*当前yaw角*/
            if (turn_cal_flag==0) /*如果第一次调整角度，设定目标角*/
            {
                targe_degree=Yaw_berfore+turn_degree;
                turn_cal_flag=1;
            }
            omega_bais=Get_BisaDegree(targe_degree, Yaw_now);
            if (fabsf(omega_bais) < 3.0f)/*到达targe_degree+-3°，停止*/ 
            {
                Speed_setnow=0;
                turn_omega_set=0;
            }/*如果到指定角度还要走改这里*/
            else 
            {
                App_Control_SetTurnSpeed(targe_degree);
                App_Control_SetMoveSpeed(run_speed);
                if (a_Flag==0) 
                {
                    if (a_thrid_Flag<3) 
                    {
                        Speed_setnow+=0.3;
                    }
                    else 
                    {
                        Speed_setnow+=0.5;
                    }
                    if(Speed_setnow>=speed_set) 
                    {
                        Speed_setnow=speed_set;
                        a_Flag=1;
                    }
                }
                // PID_LimitConfig(&pid_turn, Speed_setnow/5.0f,Speed_setnow/5.0f);
                /*限幅为当前速度值的1/5*/
                PID_LimitConfig(&pid_turn, 2.0f,-2.0f);
                turn_omega_set+=PID_Compute_Incremental(&pid_turn, Yaw_now);
                if (turn_omega_set>pid_turn.UpperLimit) turn_omega_set=pid_turn.UpperLimit;
                if (turn_omega_set<pid_turn.LowerLimit) turn_omega_set=pid_turn.LowerLimit;
            }
            // App_Motor_SetOmega_L(Speed_setnow+(turn_omega_set*Speed_setnow/speed_set));
            // App_Motor_SetOmega_R(Speed_setnow-(turn_omega_set*Speed_setnow/speed_set));
            App_Motor_SetOmega_L(Speed_setnow+(turn_omega_set));
            App_Motor_SetOmega_R(Speed_setnow-(turn_omega_set));
            break;

            case 3:/*循迹*/
            if(set_flag==0)/*首次进入循迹状态设定sp*/
            {
                PID_Pos_ChangeSP(&pid_follow, pos_set);
                set_flag=1;
            }
            //加速代码
            App_Control_SetMoveSpeed(run_speed);
            if (a_Flag==0) 
            {
                 if (a_thrid_Flag<6) 
                    {
                        Speed_setnow+=0.2;
                    }
                    else 
                    {
                        Speed_setnow+=1.5;
                    }
                    if(Speed_setnow>=speed_set) 
                    {
                        Speed_setnow=speed_set;
                        a_Flag=1;
                    }
            }
            float Pos_now=Get_Pos();/*当前位置作为循迹环fb*/
            float diff_omega=PID_Compute_Pos(&pid_follow, Pos_now);

            uint8_t turn_state=Get_turn_Flag_l();
            if (turn_state) {
                Speed_setnow=0.0;
                a_thrid_Flag=0;
                turn_times++;
                turn_now=1;
                a_Flag=0;
            }
            uint64_t pos_l=App_Encoder_GetPos_L();
            uint64_t pos_r=App_Encoder_GetPos_R();
            if(Laps==5) Laps=4.80;
            if(Laps==4) Laps=3.85;
            if(Laps==3) Laps=2.90;
            if(Laps==2) Laps=1.95;
            if((turn_times>=8*Laps&&pos_l>5820*Laps&&pos_r>7000*Laps)||(pos_l>6000*Laps&&pos_r>7200*Laps))
            {
                Speed_setnow=0.0;
                diff_omega=0.0;
            }
            if(turn_now==1)
            {
                App_Motor_SetOmega_L(0);
                App_Motor_SetOmega_R(0.75);
                turn_now=0;
            }
            else 
            {
            App_Motor_SetOmega_L(Speed_setnow-diff_omega);
            App_Motor_SetOmega_R(Speed_setnow+diff_omega);
            }
            lasttime=currenttime;
            break;
        }
    }
    else if (cmd==0) {
        Speed_setnow=0;
        turn_omega_set=0;
    }
    
    return targe_degree;//输出设定值
}

//
// @简介：对控制系统进行复位（这里用的是S2按下复位）
//
void App_Control_Reset(void)
{
	// #1. 复位暂存的值
	lasttime = 0;
	// #2. PID控制器
	PID_Reset(&pid_turn);
    PID_Pos_Reset(&pid_follow);
    turn_omega_set=0;
    // #3.标志位
    a_Flag=0;
    a_thrid_Flag=0;
    turn_cal_flag=0;
    set_flag=0;
    slow_down_flag=0;
    lastturn_state=0;
    // #4.临时变量
    targe_degree=0.0;
    Speed_setnow=0.0;
    omega_bais=0.0;
    encoder_count=0;
    turn_times=0;
    // #5.时间变量
    lasttime=0;
}

//
// @简介：改变小车自行的速度
// @参数：MoveSpeed - 执行速度，单位是rad/s，最大转速+-30rad/s
//
void App_Control_SetMoveSpeed(float MoveSpeed)
{
    if(MoveSpeed>=30.0f)
        speed_set=30.0f;
    else if (MoveSpeed<-30.0f)
        speed_set=-30.0f;
    else
        speed_set=MoveSpeed;
}

//
// @简介：改变小车运行的角度（朝哪个方向走）
// @参数：Turnspeed 两轮轮速差，单位为：rad/s
//
void App_Control_SetTurnSpeed(float Turnspeed)
{
	PID_ChangeSP(&pid_turn, Turnspeed);
}

//
// @简介：检测小车当前yaw角（执行proc前检测，这里用的是S2按下）
// @参数：cmd 是否检测；0-不检测，1-检测
//
void App_Control_SetYaw_Before(uint8_t cmd)
{
    if(cmd==1)
        Yaw_berfore=bno08x_data.yaw;
}

//
// @简介：计算目标角与当前角度的误差
// @说明：bno08x返回的yaw范围是[-180°~+180°]，所以需要这个函数
//
float Get_BisaDegree(float targe,float current)
{
    float diff=targe-current;
    if(diff > 180.0f)  diff -= 360.0f;
    if(diff < -180.0f) diff += 360.0f;
    return diff;
}
