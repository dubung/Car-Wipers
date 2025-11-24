#ifndef __MY_Servo_H__
#define __MY_Servo_H__
#include "stm32f4xx_hal.h"

typedef struct _servo
{
	
	TIM_HandleTypeDef *htimServo;
	
	uint8_t Max_Angle;
	uint8_t Min_Angle;
	uint32_t Max_PWM;
	uint32_t Min_PWM;
	uint32_t channel;
	
	
	void (*Set_Range)(struct _servo*, uint8_t,uint8_t,uint32_t,uint32_t);
	void (*Set_angle)(struct _servo*, uint8_t);
	
}Servo;

void init_Servo(Servo* servo, uint32_t ch, TIM_HandleTypeDef* htim);


#endif // __MY_Servo_H__