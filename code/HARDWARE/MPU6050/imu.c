#include "imu.h"
#include "imu_driver.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"

#include "math.h"
#include "usart.h"


volatile float exInt, eyInt, ezInt;  // ������
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
volatile MPU6050_REAL_DATA   MPU6050_Real_Data;

volatile float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz,halfT;   //��������ڴ��ļ���

static volatile float q[4]; //����Ԫ��
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
volatile float angle[3] = {0};
volatile float yaw_temp,pitch_temp,roll_temp;
volatile float last_yaw_temp,last_pitch_temp,last_roll_temp;
volatile float yaw_angle ,pitch_angle,roll_angle; //ʹ�õ��ĽǶ�ֵ

extern IMUDataTypedef imu_data;//�����ȫ�ֱ���

// Fast inverse square-root  ����ƽ������֮һ
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x; 
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
///**************************ʵ�ֺ���********************************************
//*����ԭ��:	   void IMU_getValues(volatile float * values)
//*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
//��������� �������ŵ������׵�ַ
//���ٶ�ֵ��ԭʼ���ݣ�-8192-+8192
//���ٶ�ֵ��deg/s
//������ֵ��ԭʼ����
//���������û��
//*******************************************************************************/
//void IMU_getValues(volatile float * values) {  
//		int16_t accgyroval[6];
//	//��ȡ���ٶȺ������ǵĵ�ǰADC
//  IMU_Get_Data(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
//	values[0] = accgyroval[0]/4096.0f;
//	values[1] = accgyroval[1]/4096.0f;
//	values[2] = accgyroval[2]/4096.0f;
//	values[3] = accgyroval[3]/938.8032f;
//	values[4] = accgyroval[4]/938.8032f;
//	values[5] = accgyroval[5]/938.8032f;
//    HMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
////		MPU6050_Raw_Data.Mag_X = values[6];
////		MPU6050_Raw_Data.Mag_Y = values[7];
////		MPU6050_Raw_Data.Mag_Z = values[8];
//}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
���������   ��ǰ�Ĳ���ֵ��
���������   û��
*******************************************************************************/
#define Kp 6.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki -0.1f   // integral gain governs rate of convergence of gyroscope biases
//float ki=-0.1,kp=5;
void IMU_AHRSupdate(void) {
		float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz;
		float wx, wy, wz;
    float ex, ey, ez;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

//    gx = 0.01f;
//    gy = 0.01f;
//		gz = 0.01f;
//    ax = 0.01f;
//    ay = 0.01f;
//    az = 1.0f;

    gx = Gyro[0]/180*3.14;
    gy = Gyro[1]/180*3.14;
	gz = Gyro[2]/180*3.14;
    ax = Acc[0];
    ay = Acc[1];
    az = Acc[2];
		
//		printf("%f  %f  %f  ggg%f  %f  %f\r\n",ax,ay,az,gx,gy,gz);
		
    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
//		printf("%d",now);
//		if(now>=lastUpdate)			
//    {
//        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
//    }
	if(now < lastUpdate)  //��ʱ��������ˡ�
	{
		halfT =  ((float)(now + (0xffff - lastUpdate)) / 2000.0f);   //   9000000/9000=1000(TIM2)    1000/2000 =0.5s
		lastUpdate = now;
	}
	else
	{
		halfT =  ((float)(now - lastUpdate) / 2000.0f);
	}
    lastUpdate = now;	//����ʱ��
    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
//		printf("%f ",az);	
    //�ѼӼƵ���ά����ת�ɵ�λ������
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)�����vx��vy��vz����ʵ���ǵ�ǰ�Ļ����������ϵ�ϣ����������������λ������(�ñ�ʾ������̬����Ԫ�����л���)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
		//ex ey ezΪ ax��vx�����
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;//����������л���
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // �ò���������PI����������ƫ
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getQ(float * q)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��
*******************************************************************************/
void IMU_getQ(volatile float * q) {

//    IMU_getValues(mygetqval);	 //��ȡԭʼ����,���ٶȼƺʹ�������ԭʼֵ��������ת������deg/s
    IMU_AHRSupdate();
    q[0] = q0; //���ص�ǰֵ
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void IMU_getYawPitchRoll(volatile float * angles) 
{  
    IMU_getQ(q); //����ȫ����Ԫ��
    //��Ԫ��ת����ŷ���ǣ��������Ǻ������㼴��
    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  
//		printf("%f\r\n",q[0]);
}

static int yaw_count = 0;
void GetPitchYawGxGyGz()
{
	MPU6050_Real_Data.Gyro_X = mygetqval[3];
	MPU6050_Real_Data.Gyro_Y = -mygetqval[4];
	MPU6050_Real_Data.Gyro_Z = mygetqval[5];

	last_yaw_temp = yaw_temp;
	yaw_temp = angle[0]; 
	if(yaw_temp-last_yaw_temp>=330)  //yaw��ǶȾ����������������
	{
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-330)
	{
		yaw_count++;
	}
	yaw_angle = yaw_temp + yaw_count*360;  //yaw��Ƕ�
	pitch_angle = angle[1];
	roll_angle = angle[2];	
//	printf("%.2f  %.2f  %.2f  ",	MPU6050_Real_Data.Gyro_X,MPU6050_Real_Data.Gyro_Y,MPU6050_Real_Data.Gyro_Z);
}



