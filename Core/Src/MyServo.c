#include "MyServo.h"

void FSet_Angle(Servo* myservo, uint8_t angle);
void FSet_Range(Servo* myservo, uint8_t minA, uint8_t maxA, uint32_t minP, uint32_t maxP);



void init_Servo(Servo* servo, uint32_t ch, TIM_HandleTypeDef* htim){
	
	servo->Set_angle = FSet_Angle;
	servo->Set_Range = FSet_Range;
	
	servo->channel = ch;
	servo->htimServo = htim;
	HAL_TIM_PWM_Start(servo->htimServo, servo->channel);
}

void FSet_Angle(struct _servo* myservo, uint8_t angle){

	float angleRange = myservo->Max_Angle - myservo->Min_Angle;
	float PWMRange = myservo->Max_PWM - myservo->Min_PWM;
	float Change = PWMRange/ angleRange;
	
	if(angle > myservo->Max_Angle){
		angle = myservo->Max_Angle;
	}
	
	if(angle < myservo->Min_Angle){
		angle = myservo->Min_Angle;
	}
	
	__HAL_TIM_SET_COMPARE(myservo->htimServo, myservo->channel,(Change * angle) + myservo->Min_PWM);
	
}

void FSet_Range(struct _servo* myservo, uint8_t minA, uint8_t maxA, uint32_t minP, uint32_t maxP){

	myservo->Min_Angle = minA;
	myservo->Max_Angle = maxA;
	myservo->Min_PWM = minP;
	myservo->Max_PWM = maxP;
	
}

