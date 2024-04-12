/**
  *@file imu.h
  *@date 2017-3-18
  *@author �껪ǳ��
  *@brief imu��̬����
  */
  
#ifndef _IMU_H
#define _IMU_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "math.h"
#define M_PI  (float)3.1415926535

void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //������̬
void GetPitchYawGxGyGz(void);
//extern int16_t MPU6050_FIFO[6][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
//extern int16_t HMC5883_FIFO[3][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�

extern volatile float angle[3];
extern volatile float yaw_angle,pitch_angle,roll_angle; //ʹ�õ��ĽǶ�ֵ

extern uint32_t Get_Time_Micros(void);                                                                                             


#endif

