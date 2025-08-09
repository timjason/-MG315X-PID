#ifndef ENCODER_H
#define ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

float App_Encoder_GetPos_L(void);
float App_Encoder_GetPos_R(void);
float App_Encoder_GetSpeed_L(void); 
float App_Encoder_GetSpeed_R(void); 
float App_Encoder_GetSpeed_Smoothed_L(void);
float App_Encoder_GetSpeed_Smoothed_R(void);
#endif  /* #ifndef _CLOCK_H_ */