#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"



#define    TRIG_Send(x)    x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET)
#define    TRIG_Send_2(x)  x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)

#define    ECHO_Reci        HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define    ECHO_Reci_2      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)

extern TIM_HandleTypeDef TIM3_Handler;      //¶¨Ê±Æ÷¾ä±ú

void TIM3_Init(u16 arr,u16 psc);

void SRF04_init(void);

float Hcsr04GetLength(void);
float Hcsr04GetLength_2(void);



#endif





