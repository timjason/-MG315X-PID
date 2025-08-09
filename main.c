/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "main.h"
#include "pwm.h"
#include "stdio.h"
#include "string.h"
#include "encoder.h"
#include "PID.h"
#include "key.h"
#include "motor.h"
#include "control.h"
#include "follow.h"

uint8_t oled_buffer[32];
// float Omega=0.0f;
float Runspeed=0.0;
float pos_l=0.0,pos_r=0.0;


int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    NVIC_ClearPendingIRQ(GPIO_Encoder_GPIOB_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
    OLED_Init();
    App_Motor_Init();
    App_Control_Init();
    /* Don't remove this! */
    Interrupt_Init();
    OLED_ShowString(0,7,(uint8_t *)"BNO08X Demo",8);

    OLED_ShowString(0,0,(uint8_t *)" Pos",8);
    OLED_ShowString(0,2,(uint8_t *)" pos_l",8);
    OLED_ShowString(0,4,(uint8_t *)" pos_r",8);

    OLED_ShowString(16*6,7,(uint8_t *)"Index",8);
    OLED_ShowString(16*6,0,(uint8_t *)"turn",8);
    while (1) 
    {
        sprintf((char *)oled_buffer, "%3u", bno08x_data.index);
        OLED_ShowString(18*6,6,oled_buffer,8);
        float pos=Get_Pos();
        pos_l = App_Encoder_GetPos_L()*28.0f*26.0f/360.0f/2.0f;
        pos_r = App_Encoder_GetPos_R()*28.0f*26.0f/360.0f/2.0f;
        sprintf((char *)oled_buffer, "%-6.1f", pos);
        OLED_ShowString(5*8,0,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", pos_l);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", pos_r);
        OLED_ShowString(5*8,4,oled_buffer,16);

        sprintf((char *)oled_buffer, "%6d",turn_times);
        OLED_ShowString(15*6,1,oled_buffer,8);
        sprintf((char *)oled_buffer, "%6d", bno08x_data.ay);
        OLED_ShowString(15*6,2,oled_buffer,8);
        sprintf((char *)oled_buffer, "%6d", bno08x_data.az);
        OLED_ShowString(15*6,3,oled_buffer,8);
        App_Button_Proc();
        App_Motor_Proc();
        float Omega=App_Control_Proc(S2_Flag, 3, 8.0, 90.0,4.0,2);
        float pwm_duty_l=get_pwm_duty_l();
        float pwm_duty_r=get_pwm_duty_r();
        int16_t pwmcount_l=get_pwm_count_l();
        int16_t pwmcount_r=get_pwm_count_r();
        float omega_l =  App_Encoder_GetSpeed_Smoothed_L();
        float omega_r = App_Encoder_GetSpeed_Smoothed_R();

    }
}

// int fputc(int c, FILE* stream)
// {
// 	DL_UART_Main_transmitDataBlocking(UART_0_INST, c);
//     return c;
// }

// int fputs(const char* restrict s, FILE* restrict stream)
// {
//     uint16_t i, len;
//     len = strlen(s);
//     for(i=0; i<len; i++)
//     {
//         DL_UART_Main_transmitDataBlocking(UART_0_INST, s[i]);
//     }
//     return len;
// }

// int puts(const char *_ptr)
// {
// int count = fputs(_ptr, stdout);
// count += fputs("\n", stdout);
// return count;
// }
