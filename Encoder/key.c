#include "key.h"

volatile uint8_t S2_Flag = 0; 

void App_Button_Init(void)
{
    S2_Flag=0;
}

void App_Button_Proc(void)
{
    if(!DL_GPIO_readPins(GPIO_BUTTON_PORT,GPIO_BUTTON_S2_PIN))//如果按下
    {
        mspm0_delay_ms(30);
        if(!DL_GPIO_readPins(GPIO_BUTTON_PORT,GPIO_BUTTON_S2_PIN))//20ms后还是按下
        {
            if(S2_Flag==0)
            {
                App_Control_Reset();
                //App_Control_SetYaw_Before(1);
                S2_Flag=1;
                DL_GPIO_setPins(GPIO_TB6612_PORT, GPIO_TB6612_PIN_STBY_PIN);
                DL_GPIO_setPins(GPIO_LED_TEST_PORT, GPIO_LED_TEST_PIN_LED_PIN);
                App_Motor_Cmd(S2_Flag);
            }
            else if (S2_Flag==1) 
            {
                S2_Flag=0;
                App_Motor_Cmd(S2_Flag);
                DL_GPIO_clearPins(GPIO_TB6612_PORT, GPIO_TB6612_PIN_STBY_PIN);
                DL_GPIO_clearPins(GPIO_LED_TEST_PORT, GPIO_LED_TEST_PIN_LED_PIN);
                App_Control_Reset();
                Follow_Reset ();
            }

        }
    }
}

//
// @简介：按钮读取
//
uint8_t Key_Read(void)
{
    // uint8_t temp=0;
    // if (!DL_GPIO_readPins(GPIO_ControlKey_PORT,GPIO_ControlKey_PIN_choose_PIN)) //按下选题按钮
    // temp=1;
    // if (!DL_GPIO_readPins(GPIO_ControlKey_PORT,GPIO_ControlKey_PIN_setlaps_PIN)) //按下加圈按钮
    // temp=2;
    // if (!DL_GPIO_readPins(GPIO_ControlKey_PORT,GPIO_ControlKey_PIN_follow_stat_PIN)) //按下启动按钮
    // temp=3;
    // return temp;
}

uint8_t Key_Val,Key_Down,Key_Up,Key_Old;
uint8_t Key_ChooseFlag=0,Key_SetLaps_flag=0,Key_Follow_Stat_Flag=0;
volatile uint8_t CMD = 0; 
volatile uint8_t Laps = 0; 
volatile uint8_t Follow_Start = 0; 

//
// @简介：按钮进程
//
void Key_Porc(void)
{
    Key_Val=Key_Read();//读取当前键值
    Key_Down=Key_Val & (Key_Val^Key_Old);//检测下降沿
    Key_Up=~Key_Val & (Key_Val^Key_Old);//检测上升沿
    Key_Old=Key_Val;//更新

    if (Key_Down==1&&Key_ChooseFlag==0)
        Key_ChooseFlag=1;
    else if (Key_Down==1&&Key_ChooseFlag==0)
        Key_ChooseFlag=1;

    if (Key_Down==2&&Key_SetLaps_flag==0) 
        Key_SetLaps_flag=1;
    else if (Key_Down==2&&Key_SetLaps_flag==1)
        Key_SetLaps_flag=0;

    if (Key_Down==3&&Key_Follow_Stat_Flag==0) 
        Key_Follow_Stat_Flag=1;
    else if (Key_Down==3&&Key_Follow_Stat_Flag==1)
        Key_Follow_Stat_Flag=0;
    //按钮按下调整flag

    if (Key_ChooseFlag)
        CMD=1;
    else
        CMD=0;

    if(Key_SetLaps_flag)
        Laps++;

    if (Key_Follow_Stat_Flag) 
    {
        if (CMD) 
        {
            Follow_Start = 1;
        }
    }

    if(CMD==1)
    {
        App_Control_Reset();
        DL_GPIO_setPins(GPIO_TB6612_PORT, GPIO_TB6612_PIN_STBY_PIN);
        DL_GPIO_setPins(GPIO_LED_TEST_PORT, GPIO_LED_TEST_PIN_LED_PIN);
        App_Motor_Cmd(1);
    }
    else if (CMD==0) 
    {
        App_Motor_Cmd(0);
        DL_GPIO_clearPins(GPIO_TB6612_PORT, GPIO_TB6612_PIN_STBY_PIN);
        DL_GPIO_clearPins(GPIO_LED_TEST_PORT, GPIO_LED_TEST_PIN_LED_PIN);
        App_Control_Reset();
        Follow_Reset ();
    }
    
}