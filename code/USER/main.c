#include "sys.h"
#include "delay.h"
#include "lcd.h"
#include "timer.h"
#include "motor.h"

int main(void)
{
	float SRF04_Value = 0;
	float SRF04_Value_2 = 0;
	uint8_t distance[3] = {0, 0, 0};
	uint8_t distance_2[3] = {0, 0, 0};
	//蜂鸣器报警变量
	uint8_t bueezr_flag = 0,  count = 0, buzzer_time = 3;

	HAL_Init();
	Stm32_Clock_Init(RCC_PLL_MUL9);   
	delay_init(72);
	LCD_init();
	LCD_write_string(1, 0, "L:");
	LCD_write_string(7, 0, "R:");
	SRF04_init();
	motor_init();
	BUZZER = 1;
	TIM3_Init(1000-1, 72-1);
	SRF04_Value = Hcsr04GetLength()-2;
	SRF04_Value_2 = Hcsr04GetLength_2();
	while(1)
	{
		/*超声波传感器读取*/
		SRF04_Value = Hcsr04GetLength();
		SRF04_Value_2 = Hcsr04GetLength_2();

		/*数据进行处理*/
		distance[0] = (uint16_t)SRF04_Value / 100 + 48;
		distance[1] = (uint16_t)SRF04_Value % 100 / 10 + 48;
		distance[2] = (uint16_t)SRF04_Value % 100 % 10 + 48;

		distance_2[0] = (uint16_t)SRF04_Value_2 / 100 + 48;
		distance_2[1] = (uint16_t)SRF04_Value_2 % 100 / 10 + 48;
		distance_2[2] = (uint16_t)SRF04_Value_2 % 100 % 10 + 48;

		/*lcd显示*/
		LCD_write_string(3, 0, (char*)distance);
		LCD_write_string(9, 0, (char*)distance_2);

		
		/*判断距离，电机转动，进行显示*/
		if (SRF04_Value < 20 && SRF04_Value_2 > 20) { //右转
			motor_type(TURN_RIGHT);
			LCD_write_string(1, 1, "turn right  ");
		} else if (SRF04_Value > 20 && SRF04_Value_2 < 20) { //左转
			motor_type(TURN_LEFT);
			LCD_write_string(1, 1, "turn left   ");
		} else if (SRF04_Value < 20 && SRF04_Value_2 < 20) { //后退
			motor_type(RUN_BACK);
			LCD_write_string(1, 1, "run back   ");
		} else { //前进
			motor_type(RUN_FORWARD);
			LCD_write_string(1, 1, "run forward");
		}
	
		
        	//超声波值与报警值比较
		if (SRF04_Value < 20 || SRF04_Value_2 < 20) {	
			bueezr_flag = 1;
		} else {
			bueezr_flag = 0;
			BUZZER = 1;
		}
		/*******蜂鸣器报警********/
		count++;
		if (count > buzzer_time * 10)
			count = buzzer_time + 1;
		if(count % buzzer_time == 0 && bueezr_flag)
			BUZZER = ~BUZZER;//蜂鸣器取反  发出声音提示
    }
}



