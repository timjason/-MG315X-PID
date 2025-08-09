#include "follow.h"

float Pos_last=0.0;//上次检测到的位置
uint64_t lasttime=0;//上次检测的时间
uint8_t r1,r2,r3,r4,r5,r6,r7;
uint8_t turn_Flag_r=0,turn_Flag_l=0;
uint64_t lasttime_turnleft=0,lasttime_turnright=0;
uint64_t currenttime_turn_left=0,currenttime_turn_right=0;
//
// @简介：获取当前黑线所在的位置
// @说明：至左向右每个传感器权重为1-8
//
float Get_Pos(void)
{
    float Pos=0.0f;
    uint64_t currenttime=GetUs();
    uint8_t num=0;
    if (currenttime-lasttime<20) {
    
    }
    else 
    {
        
        if(DL_GPIO_readPins(GPIO_follow_PIN_x1_PORT,GPIO_follow_PIN_x1_PIN))
        {
            r1=1;
            num++;
        }
        else 
        {
           r1=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x2_PORT,GPIO_follow_PIN_x2_PIN))
        {
            r2=1;
            num++;
        }
        else 
        {
           r2=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x3_PORT,GPIO_follow_PIN_x3_PIN))
        {
            r3=1;
            num++;
        }
        else 
        {
           r3=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x4_PORT,GPIO_follow_PIN_x4_PIN))
        {
            r4=1;
            num++;
        }
        else 
        {
           r4=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x5_PORT,GPIO_follow_PIN_x5_PIN))
        {
            r5=1;
            num++;
        }
        else 
        {
           r5=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x6_PORT,GPIO_follow_PIN_x6_PIN))
        {
            r6=1;
            num++;
        }
        else 
        {
           r6=0; 
        }

        if(DL_GPIO_readPins(GPIO_follow_PIN_x7_PORT,GPIO_follow_PIN_x7_PIN))
        {
            r7=1;
            num++;
        }
        else 
        {
           r7=0; 
        }

        if(num==0) num=1;
        Pos=(r1*1.0f+r2*2.0f+r3*3.0f+r4*4.0f+r5*5.0f+r6*6.0f+r7*7.0f)/num;
        if (r1==1&&r2+r3+r4>=2) 
        {
            turn_Flag_l=1;
        }
        else 
        {
            turn_Flag_l=0;
        }

    }
    lasttime=currenttime;
    if(Pos!=0) Pos_last=Pos;
    if(Pos==0) Pos=Pos_last;
    return Pos;
}


// float Get_Pos_open(void)
// {
//     uint8_t pos=0;
//     uint64_t currenttime=GetUs();
//     if (currenttime-lasttime<20) {
    
//     }
//     else 
//     {
//         uint8_t num=0;
//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x1_PORT,GPIO_follow_PIN_x1_PIN))
//         {
//             r1=1;
//         }
//         else 
//         {
//            r1=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x2_PORT,GPIO_follow_PIN_x2_PIN))
//         {
//             r2=1;
//         }
//         else 
//         {
//            r2=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x3_PORT,GPIO_follow_PIN_x3_PIN))
//         {
//             r3=1;
//         }
//         else 
//         {
//            r3=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x4_PORT,GPIO_follow_PIN_x4_PIN))
//         {
//             r4=1;
//         }
//         else 
//         {
//            r4=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x5_PORT,GPIO_follow_PIN_x5_PIN))
//         {
//             r5=1;
//         }
//         else 
//         {
//            r5=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x6_PORT,GPIO_follow_PIN_x6_PIN))
//         {
//             r6=1;
//         }
//         else 
//         {
//            r6=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x7_PORT,GPIO_follow_PIN_x7_PIN))
//         {
//             r7=1;
//         }
//         else 
//         {
//            r7=0; 
//         }

//         if(!DL_GPIO_readPins(GPIO_follow_PIN_x8_PORT,GPIO_follow_PIN_x8_PIN))
//         {
//             r8=1;
//         }
//         else 
//         {
//            r8=0; 
//         }
//     if(r1==0&&r2==0&&r3==0&&r4==0&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=0;//stop
//     if(r1==0&&r2==0&&r3==0&&r4==1&&r5==1&&r6==0&&r7==0&&r8==0)
//     pos=1;//直行
//     if(r1==0&&r2==0&&r3==0&&r4==1&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=2;//小右转
//     if(r1==0&&r2==0&&r3==0&&r4==0&&r5==1&&r6==0&&r7==0&&r8==0)
//     pos=3;//小左转

//     if(r1==0&&r2==0&&r3==1&&r4==0&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=4;//右转
//     if(r1==0&&r2==0&&r3==0&&r4==0&&r5==0&&r6==1&&r7==0&&r8==0)
//     pos=5;//左转

//     if(r1==0&&r2==1&&r3==0&&r4==0&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=6;//右转
//     if(r1==0&&r2==0&&r3==0&&r4==0&&r5==0&&r6==0&&r7==1&&r8==0)
//     pos=7;//左转

//     if(r1==1&&r2==0&&r3==0&&r4==0&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=8;//大右转
//     if(r1==0&&r2==0&&r3==0&&r4==0&&r5==0&&r6==0&&r7==0&&r8==0)
//     pos=9;//大左转

//     if(r1==1&&r2==1&&r3==1&&r6==0&&r7==0&&r8==0)
//     pos=10;//直角弯
//     if(r1==0&&r2==0&&r3==0&&r6==1&&r7==1&&r8==1)
//     pos=11;//直角弯   
//     }
//     return pos;
// }

uint8_t Get_turn_Flag_l(void)
{
    return turn_Flag_l;
}
uint8_t Get_turn_Flag_r(void)
{
    return turn_Flag_r;
}

void Follow_Reset (void)
{
    lasttime_turnleft=0;
    lasttime_turnright=0;
    currenttime_turn_left=0;
    currenttime_turn_right=0;
    lasttime=0;
}
