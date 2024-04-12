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
	//��������������
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
		/*��������������ȡ*/
		SRF04_Value = Hcsr04GetLength();
		SRF04_Value_2 = Hcsr04GetLength_2();

		/*���ݽ��д���*/
		distance[0] = (uint16_t)SRF04_Value / 100 + 48;
		distance[1] = (uint16_t)SRF04_Value % 100 / 10 + 48;
		distance[2] = (uint16_t)SRF04_Value % 100 % 10 + 48;

		distance_2[0] = (uint16_t)SRF04_Value_2 / 100 + 48;
		distance_2[1] = (uint16_t)SRF04_Value_2 % 100 / 10 + 48;
		distance_2[2] = (uint16_t)SRF04_Value_2 % 100 % 10 + 48;

		/*lcd��ʾ*/
		LCD_write_string(3, 0, (char*)distance);
		LCD_write_string(9, 0, (char*)distance_2);

		
		/*�жϾ��룬���ת����������ʾ*/
		if (SRF04_Value < 20 && SRF04_Value_2 > 20) { //��ת
			motor_type(TURN_RIGHT);
			LCD_write_string(1, 1, "turn right  ");
		} else if (SRF04_Value > 20 && SRF04_Value_2 < 20) { //��ת
			motor_type(TURN_LEFT);
			LCD_write_string(1, 1, "turn left   ");
		} else if (SRF04_Value < 20 && SRF04_Value_2 < 20) { //����
			motor_type(RUN_BACK);
			LCD_write_string(1, 1, "run back   ");
		} else { //ǰ��
			motor_type(RUN_FORWARD);
			LCD_write_string(1, 1, "run forward");
		}
	
		
        	//������ֵ�뱨��ֵ�Ƚ�
		if (SRF04_Value < 20 || SRF04_Value_2 < 20) {	
			bueezr_flag = 1;
		} else {
			bueezr_flag = 0;
			BUZZER = 1;
		}
		/*******����������********/
		count++;
		if (count > buzzer_time * 10)
			count = buzzer_time + 1;
		if(count % buzzer_time == 0 && bueezr_flag)
			BUZZER = ~BUZZER;//������ȡ��  ����������ʾ
    }
}



