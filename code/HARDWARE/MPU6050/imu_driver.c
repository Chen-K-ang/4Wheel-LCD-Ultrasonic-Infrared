///**
//  *@file test_imu.c
//  *@date 2016-12-12
//  *@author Albert.D
//  *@brief 
//  */
//  
//#include "imu.h"
//#include "imu_driver.h"
//#include "mpu6500_reg.h"
//#include "IST8310_reg.h"



//uint8_t MPU_id = 0;

//volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;    //原始数据
//volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
//AHRS ahrs;

//IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
//IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

//int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
//int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
//int16_t gxset=8,gyset=0,gzset=-11;
//int16_t axset=50,ayset=-70,azset=0;

////int16_t gxset=0,gyset=0,gzset=0;
////int16_t axset=0,ayset=0,azset=0;
//int16_t mxset=0,myset=0,mzset=0;
////int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;

////uint8_t *imu_buf = (uint8_t*)&imu_data;  //把结构体强制类型转化为数组


////Write a register to MPU6500
//uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
//{
//  static uint8_t MPU_Rx, MPU_Tx;
//  
//  MPU6500_NSS_Low();
//  
//  MPU_Tx = reg&0x7f;
//  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//  MPU_Tx = data;
//  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//  
//  MPU6500_NSS_High();
//  return 0;
//}

////Read a register from MPU6500
//uint8_t MPU6500_Read_Reg(uint8_t const reg)
//{
//  static uint8_t MPU_Rx, MPU_Tx;
//  
//  MPU6500_NSS_Low();
//  
//  MPU_Tx = reg|0x80;
//  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//  
//  MPU6500_NSS_High();
//  return MPU_Rx;
//}

////Read registers from MPU6500,address begin with regAddr  
//uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
//{
//  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
//  MPU6500_NSS_Low();
//  
//  MPU_Tx = regAddr|0x80;
//  MPU_Tx_buff[0] = MPU_Tx;
//  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
//  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
//  
//  MPU6500_NSS_High();
//  return 0;
//}

////Write IST8310 register(寄存器) through MPU6500  写磁力计
//static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
//{
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
//  HAL_Delay(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
//  HAL_Delay(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
//  HAL_Delay(1);
//}

////Read IST8310 register through MPU6500    读磁力计数值返回值为数据
///**************************实现函数********************************************
//*函数原型:	  static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
//*功　　能:	  通过MPU6500读取IST8310的数值
//输入参数：    IST8310的地址
//输出参数：    地址对应数据
//*******************************************************************************/
//static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
//{
//  uint8_t data;
//	
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
//  HAL_Delay(1);
////	delay_us(500);
////	delay_ms(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
//	HAL_Delay(1);
//  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
//	
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
//	HAL_Delay(1);

//  return data;
//}

////Initialize the MPU6500 I2C Slave0 for I2C reading
//static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
//{
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
//  HAL_Delay(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
//  HAL_Delay(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
//  HAL_Delay(1);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
//  HAL_Delay(1);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
//  HAL_Delay(1);
//}

////Initialize the MPU6500  初始化MPU6500  输出 加速度 角速度
//uint8_t MPU6500_Init(void)
//{
//  uint8_t index = 0;
//  uint8_t MPU6500_Init_Data[20][2] = 
//  {
//    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
//    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
//		{MPU6500_INT_PIN_CFG,   0x10},
//		{MPU6500_INT_ENABLE,    0x01},
//    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro  加速度输出
//		{MPU6500_SMPLRT_DIV,    0x00},
//    {MPU6500_CONFIG,        0x02},      // LPF 98Hz  低通滤波器
//    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps  角速度
//    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G  加速度
//    {MPU6500_ACCEL_CONFIG_2,0x00},     
//		{MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
//		{MPU6500_USER_CTRL,     0x00},     
//    {MPU6500_USER_CTRL,     0x10},    
//    {MPU6500_USER_CTRL,     0x30},      
//  };
//  
//  HAL_Delay(100);
//  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
//  
//  for(index = 0; index < 20; index++)
//  {
//    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
//    HAL_Delay(1);
//  }
//	printf("MPU_6500_init_success\r\n");
//  return 0;
//}
////Initialize the IST8310  初始化磁力计
//uint8_t IST8310_Init(void)
//{
//  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
//  HAL_Delay(10);
//  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
//  HAL_Delay(10);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
//  HAL_Delay(10);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
//  HAL_Delay(10);
// 
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);//初始化磁力计
//	
//  IST_Reg_Write_By_MPU(MPU6500_I2C_SLV1_REG, 0x01);
//	
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
//	
//  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
//    return 1; //error
//  HAL_Delay(10);
//	
//  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
//  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
//    return 2;
//  HAL_Delay(10);

//  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
//  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
//    return 3;
//  HAL_Delay(10);
//  
//  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
//  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
//    return 4;
//  HAL_Delay(10);
//  
//  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
//  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
//    return 5;
//  HAL_Delay(10);
//  
//  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
//  HAL_Delay(10);
//  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
//  HAL_Delay(10);
// 	
//	printf("ist_8310_init_succes\r\n");
//	
//  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
//  HAL_Delay(100);
//  return 0;
//}

////Set the accelerated velocity resolution  设置加速度分辨率
//uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
//{
//  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
//}

////Set the angular velocity resolution  设置角速度分辨率
//uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
//{
//  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
//}


///**************************实现函数********************************************
//*函数原型:	  void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
//*功　　能:	  将MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz去10次平均数后处理后存储 加速度  角速度
//*输入参数：   存在MPU6050_FIFO[i][10]
//*输出参数：    
//*******************************************************************************/
//void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
//{
//	uint8_t i = 0;
//	int32_t sum=0;
//	
//	for(i=1;i<10;i++)
//	{
//		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
//		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
//		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
//		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
//		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
//		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
//	}
//	
//	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
//	MPU6050_FIFO[1][9]=ay;
//	MPU6050_FIFO[2][9]=az;
//	MPU6050_FIFO[3][9]=gx;
//	MPU6050_FIFO[4][9]=gy;
//	MPU6050_FIFO[5][9]=gz;
//	
//	for(i=0;i<10;i++)//求当前数组的合，再取平均值
//	{	
//		 sum+=MPU6050_FIFO[0][i];
//	}
//	MPU6050_FIFO[0][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=MPU6050_FIFO[1][i];
//	}
//	MPU6050_FIFO[1][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=MPU6050_FIFO[2][i];
//	}
//	MPU6050_FIFO[2][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=MPU6050_FIFO[3][i];
//	}
//	MPU6050_FIFO[3][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=MPU6050_FIFO[4][i];
//	}
//	MPU6050_FIFO[4][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=MPU6050_FIFO[5][i];
//	}
//	MPU6050_FIFO[5][10]=sum/10;
//	
//}


////Get 6 axis data from MPU6500
//void IMU_Get_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
//{
//  uint8_t mpu_buff[14];
//  if(isMPU6050_is_DRY)//MPU触发1KHz外部中断
//  {
//	isMPU6050_is_DRY = 0;
//    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
//    imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];  //加速度
//    imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
//    imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
//  
//    imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
//  
//    imu_data.gx = mpu_buff[8]<<8  |mpu_buff[9]  - imu_data_offest.gx;  //角速度
//    imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
//    imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
//	
//	MPU6050_DataSave(imu_data.ax,imu_data.ay,imu_data.az,imu_data.gx,imu_data.gy,imu_data.gz);  		
//	*ax = MPU6050_FIFO[0][10]-axset;//=MPU6050_FIFO[0][10];
//	*ay = MPU6050_FIFO[1][10]-ayset;//=MPU6050_FIFO[1][10];
//	*az = MPU6050_FIFO[2][10]-azset;//=MPU6050_FIFO[2][10];
//	*gx = MPU6050_FIFO[3][10]-gxset;
//	*gy = MPU6050_FIFO[4][10]-gyset;
//	*gz = MPU6050_FIFO[5][10]-gzset;
////		*gx = MPU6050_FIFO[3][10] - GyroSavedCaliData.GyroXOffset;
////		*gy = MPU6050_FIFO[4][10] - GyroSavedCaliData.GyroYOffset;
////		*gz = MPU6050_FIFO[5][10] - GyroSavedCaliData.GyroZOffset;
//	}
//	else
//	{       //读取上一次的值
//		*ax = MPU6050_FIFO[0][10]-axset;//=MPU6050_FIFO[0][10];
//		*ay = MPU6050_FIFO[1][10]-ayset;//=MPU6050_FIFO[1][10];
//		*az = MPU6050_FIFO[2][10]-azset;//=MPU6050_FIFO[2][10];
//		*gx = MPU6050_FIFO[3][10]-gxset;
//		*gy = MPU6050_FIFO[4][10]-gyset;
//		*gz = MPU6050_FIFO[5][10]-gzset;
//	}
////	printf("%d  %d  %d  %d  %d  %d\r\n",*gx,*gy,*gz,*ax,*ay,*az);
////	printf("%d %d %d    \r\n",gxset,gyset,gzset);
////	printf("%d %d %d  %d %d %d\r\n  ",*gx,*gy,*gz,imu_data.gx ,imu_data.gy,imu_data.gz);
//}


////Get Geomagnetic data from IST8310
//void IST_Get_Data(int16_t *x,int16_t *y,int16_t *z)
//{
//	uint8_t ist_buf[6];
////		ist_buf[0] = IST_Reg_Read_By_MPU(IST8310_WHO_AM_I);//测试是否能够读取
//	ist_buf[0] = IST_Reg_Read_By_MPU(IST8310_R_XL);
//	ist_buf[1] = IST_Reg_Read_By_MPU(IST8310_R_XM);
//	ist_buf[2] = IST_Reg_Read_By_MPU(IST8310_R_YL);
//	ist_buf[3] = IST_Reg_Read_By_MPU(IST8310_R_YM);
//	ist_buf[4] = IST_Reg_Read_By_MPU(IST8310_R_ZL);
//	ist_buf[5] = IST_Reg_Read_By_MPU(IST8310_R_ZM);

//	imu_data.mx = ist_buf[0]<<8 |ist_buf[1] - imu_data_offest.mx;  //地磁数据
//	imu_data.my = ist_buf[2]<<8 |ist_buf[3] - imu_data_offest.my;
//	imu_data.mz = ist_buf[4]<<8 |ist_buf[5] - imu_data_offest.mz;	
//	HMC58X3_newValues(imu_data.mx,imu_data.my,imu_data.mz);
//	*x = HMC5883_FIFO[0][10];
//	*y = HMC5883_FIFO[1][10];
//	*z = HMC5883_FIFO[2][10];
//}
///**************************实现函数********************************************
//*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
//*功　　能:	   更新一组数据到FIFO数组
//输入参数：  磁力计三个轴对应的ADCx值
//输出参数：  无
//*******************************************************************************/
//void HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
//{
//	uint8_t i = 0;
//	int32_t sum=0;

//	for(i=1;i<10;i++)
//	{
//		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
//		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
//		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
//	}
//	HMC5883_FIFO[0][9]= x;//将新的数据放置到 数据的最后面
//	HMC5883_FIFO[1][9]= y;
//	HMC5883_FIFO[2][9]= z;
//	
//	for(i=0;i<10;i++)//求当前数组的合，再取平均值
//	{	
//		 sum+=HMC5883_FIFO[0][i];
//	}
//	HMC5883_FIFO[0][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=HMC5883_FIFO[1][i];
//	}
//	HMC5883_FIFO[1][10]=sum/10;

//	sum=0;
//	for(i=0;i<10;i++){
//		 sum+=HMC5883_FIFO[2][i];
//	}
//	HMC5883_FIFO[2][10]=sum/10;
//	//以上全部为未校准数据
////	if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
////	{
////		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
////	}
////	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
////	{
////		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
////	}
////	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
////	{
////		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
////	}

////	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
////	{
////		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
////	}
////	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
////	{
////		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
////	}
////	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
////	{
////		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
////	}		
//}
///**************************实现函数********************************************
//*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
//*功　　能:	   读取 磁力计的当前ADC值
//输入参数：    三个轴对应的输出指针	
//输出参数：  无
//*******************************************************************************/
//void HMC58X3_getlastValues(int16_t *x,int16_t *y,int16_t *z) 
//{
//    *x = HMC5883_FIFO[0][10];
//    *y = HMC5883_FIFO[1][10]; 
//    *z = HMC5883_FIFO[2][10]; 
//}
///**************************实现函数********************************************
//*函数原型:	  void HMC58X3_mgetValues(volatile float *arry)
//*功　　能:	   读取 校正后的 磁力计ADC值
//输入参数：    输出数组指针	
//输出参数：    无
//*******************************************************************************/
//void HMC58X3_mgetValues(volatile float *arry) 
//{
//    int16_t xr,yr,zr;
//		IST_Get_Data(&xr, &yr, &zr);
//		arry[0] = xr*1.0f;
//		arry[1] = yr*1.0f;
//		arry[2] = zr*1.0f;
//	
////    HMC58X3_getRaw(&xr, &yr, &zr);
////    arry[0]= HMC5883_lastx=((float)(xr - MagSavedCaliData.MagXOffset)) * MagSavedCaliData.MagXScale;
////    arry[1]= HMC5883_lasty=((float)(yr - MagSavedCaliData.MagYOffset)) * MagSavedCaliData.MagYScale;
////    arry[2]= HMC5883_lastz=((float)(zr - MagSavedCaliData.MagZOffset)) * MagSavedCaliData.MagZScale;
//}

