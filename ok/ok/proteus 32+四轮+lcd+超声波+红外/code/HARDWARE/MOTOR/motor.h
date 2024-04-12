#ifndef _MOTOR_H
#define _MOTOR_H

#include "sys.h"
#define    LEFT_A(x)     x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)
#define    LEFT_B(x)     x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define    RIGHT_A(x)    x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET)
#define    RIGHT_B(x)    x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET):  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET)
#define    BUZZER        PBout(13)

#define TURN_LEFT 0x00
#define TURN_RIGHT 0x01
#define RUN_BACK 0x02
#define RUN_FORWARD 0x03
void motor_init(void);
void motor_type(char type);
#endif
