#ifndef _FOLLOW_H_
#define _FOLLOW_H_

#include "ti_msp_dl_config.h"
#include "clock.h"

float Get_Pos();
uint8_t Get_turn_Flag_l(void);
uint8_t Get_turn_Flag_r(void);
void Follow_Reset (void);
// float Get_Pos_open(void);

#endif