#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "encoder.h"

static volatile int encoder_l = 0; // 左电机编码器的值
static volatile int encoder_r = 0; // 右电机编码器的值
static volatile int8_t direction_l = 1; // 左电机旋转的方向，1 - 正转，-1 - 反转
static volatile int8_t direction_r = 1; // 右电机旋转的方向，1 - 正转，-1 - 反转
static volatile uint64_t t0_l = 0, t1_l = 0; // 左电机编码器发生变化的时间，单位us
static volatile uint64_t t0_r = 0, t1_r = 0; // 右电机编码器发生变化的时间，单位us
float speed_l_filtered = 0.0f;
float speed_r_filtered = 0.0f;
const float alpha = 0.16f;  // 滤波系数（0~1，越小越平滑）



float App_Encoder_GetSpeed_Smoothed_L(void) {
    float raw_speed = App_Encoder_GetSpeed_L();
    speed_l_filtered = alpha * raw_speed + (1 - alpha) * speed_l_filtered;
    return speed_l_filtered;
}

float App_Encoder_GetSpeed_Smoothed_R(void) {
    float raw_speed = App_Encoder_GetSpeed_R();
    speed_r_filtered = alpha * raw_speed + (1 - alpha) * speed_r_filtered;
    return speed_r_filtered;
}


// @简介：读取左轮胎旋转的角度，单位：度

float App_Encoder_GetPos_L(void)
{
	return encoder_l / 26.0f / 28.0f * 360.0f * 2.0f; 
}

//
// @简介：读取右轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_R(void)
{
	return encoder_r / 26.0f / 28.0f * 360.0f * 2.0f; 
}

//
// @简介：读取左轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_L(void)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t direction_cpy = direction_l;
	uint64_t t0_cpy = t0_l;
	uint64_t t1_cpy = t1_l;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(direction_cpy == +2 || direction_cpy == -2) return 0.0f;
	
	uint64_t now = GetUs();
	
	float T;
	
	if(t0_cpy - t1_cpy > now - t0_cpy)
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}
	else
	{
		T = (now - t0_cpy) * 1.0e-6f;
    }
	return direction_cpy / T / 26.0f / 28.0f * 6.28f * 2.0f;
}


//
// @简介：读取右轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_R(void)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t direction_cpy = direction_r;
	uint64_t t0_cpy = t0_r;
	uint64_t t1_cpy = t1_r;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(direction_cpy == +2 || direction_cpy == -2) return 0.0f;
	
	// uint64_t now = GetUs();
    uint64_t now =  GetUs();
	
	float T;
	
	if(t0_cpy - t1_cpy > now - t0_cpy)
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}
	else
	{
		T = (now - t0_cpy) * 1.0e-6f;
	}
	
	return direction_cpy / T / 26.0f / 28.0f * 6.28f * 2.0f;
}
	
void GROUP1_IRQHandler(void)
{

    //torris
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_Encoder_E1A_PIN);
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_Encoder_E2A_PIN);

if (gpioA & GPIO_Encoder_E1A_PIN) {   
    t1_l = t0_l;
    t0_l = GetUs();
    if(DL_GPIO_readPins(GPIO_Encoder_E1B_PORT,GPIO_Encoder_E1B_PIN)) { // 前
        encoder_l++;
        if(direction_l < 0) { // 之前轮胎是反转
            direction_l = +2;
        } else {
            direction_l = 1;
        }
    } else { // 后(反转)
        encoder_l--;
        if(direction_l > 0) { // 之前轮胎是正转
            direction_l = -2;
        } else {
            direction_l = -1;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIO_Encoder_E1A_PORT, GPIO_Encoder_E1A_PIN);
}

// 右编码器
if (gpioB & GPIO_Encoder_E2A_PIN) {       
    t1_r = t0_r;
    t0_r = GetUs();
    if(DL_GPIO_readPins(GPIO_Encoder_E2B_PORT,GPIO_Encoder_E2B_PIN)) { // 后
        encoder_r--;
        if(direction_r > 0) { // 之前轮胎是正转
            direction_r = -2;
        } else {
            direction_r = -1;
        }
    } else { // 前
        encoder_r++;
        if(direction_r < 0) { // 之前轮胎是反转
            direction_r = +2;
        } else {
            direction_r = 1;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIO_Encoder_E2A_PORT,GPIO_Encoder_E2A_PIN);
}

	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) 
    {
        #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                // #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
                // case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                //     Read_VL53L0X();
                //     break;
                // #endif

                default:
                    break;
            }
        #endif

        #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB))
            {
                #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
                case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
                    Read_Quad();
                    break;
                #endif

                #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
                case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                    Read_LSM6DSV16X();
                    break;
                #endif

                #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
                #endif

                default:
                    break;
            }
        #endif

        #if defined GPIO_MPU6050_INT_IIDX
            case GPIO_MPU6050_INT_IIDX:
                Read_Quad();
                break;
        #endif
    }
}

int fputc(int c, FILE* stream)
{
	DL_UART_Main_transmitDataBlocking(UART_0_INST, c);
    return c;
}

int fputs(const char* restrict s, FILE* restrict stream)
{
    uint16_t i, len;
    len = strlen(s);
    for(i=0; i<len; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, s[i]);
    }
    return len;
}

int puts(const char *_ptr)
{
int count = fputs(_ptr, stdout);
count += fputs("\n", stdout);
return count;
}

