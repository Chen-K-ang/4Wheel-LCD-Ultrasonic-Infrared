#include "motor.h"

void motor_init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOB_CLK_ENABLE();          

	GPIO_Initure.Pin=GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13; 
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_Initure);
}

void motor_type(char type)
{
	switch (type) {
		case TURN_LEFT:
			LEFT_A(0);LEFT_B(1); 
		        RIGHT_A(0);RIGHT_B(1);
			break;
		case TURN_RIGHT:
			LEFT_A(1);LEFT_B(0); 
		        RIGHT_A(1);RIGHT_B(0);
			break;
		case RUN_BACK:
			LEFT_A(0);LEFT_B(1); 
		        RIGHT_A(1);RIGHT_B(0);
			break;
		case RUN_FORWARD:
			LEFT_A(1);LEFT_B(0); //正转
		        RIGHT_A(0);RIGHT_B(1); //正转
			break;
		default:
			break;
	}
}
