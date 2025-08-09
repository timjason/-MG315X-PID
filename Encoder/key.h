#ifndef _KEY_H
#define _KEY_H

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "motor.h"
#include "control.h"
#include "follow.h"

extern volatile uint8_t S2_Flag; 
extern volatile uint8_t CMD ; 
extern volatile uint8_t Laps ; 
extern volatile uint8_t Follow_Start ; 
void App_Button_Proc(void);
void App_Button_Init(void);
uint8_t Key_Read(void);
void Key_Porc(void);

#endif