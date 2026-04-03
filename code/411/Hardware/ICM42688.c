#include "ICM42688.h"
#include "math.h"


#define ICM42688_Write_8BIT(reg_address,write8bit)\
				HAL_I2C_Mem_Write(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), (reg_address), I2C_MEMADD_SIZE_8BIT, &(write8bit), 1, HAL_MAX_DELAY)

#define ICM42688_Read_8BIT(reg_address, read8bit)\
				HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), (reg_address), I2C_MEMADD_SIZE_8BIT, &(read8bit), 1, HAL_MAX_DELAY)

#define ICM42688_BANK_SWITCH(x)	ICM42688_Write_8BIT(ICM42688_REG_BANK_SEL, (x))

#define PI 3.1415926f

#define GYRO_OFFSET_LSB_PER_DPS     (0.03125f)   // 1/32 dps
#define GYRO_OFFSET_MAX             (2047)
#define GYRO_OFFSET_MIN             (-2048)

#define ACCEL_OFFSET_LSB_PER_G      (0.00049f)   // 1g/2048 ≈ 0.00049
#define ACCEL_OFFSET_MAX            (2047)
#define ACCEL_OFFSET_MIN            (-2048)
float gyro_curr_resolution = 0;
float accel_curr_resolution = 0;

/* @brief: 切换到指定寄存器组
   @param: new_bank - 目标组号 (0-3)
   @return: 无
 */
void ICM_BANK_SWITCH(uint8_t new_bank)
{
	uint8_t reg;
	HAL_StatusTypeDef err;
	err = ICM42688_Read_8BIT(ICM42688_REG_BANK_SEL, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg &= ~ICM42688_BANK_SEL_Msk;  
	reg |= (new_bank & ICM42688_BANK_SEL_Msk); 
	err = ICM42688_Write_8BIT(ICM42688_REG_BANK_SEL, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
}

/* @brief: 获取传感器原始数据平均值
   @param: data - 输出平均值数据
   @param: times - 采样次数
   @return: 无
 */
void ICM_GET_Average_Raw_data(ICM42688_Raw_Data_t* data, uint16_t times)
{
	ICM42688_Raw_Data_t raw_data;
	int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
   int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
	for(uint8_t i = 0; i < times; i++){
		ICM42688_ReadSensorRawData(&raw_data);
		sum_gx +=raw_data.gyro_x;
		sum_gy +=raw_data.gyro_y;
		sum_gz +=raw_data.gyro_z;
		sum_ax +=raw_data.accel_x;
		sum_ay +=raw_data.accel_y;
		sum_az +=raw_data.accel_z;
	}
	data->gyro_x = sum_gx / times;
	data->gyro_y = sum_gy / times;
	data->gyro_z = sum_gz / times;
	data->accel_x = sum_ax / times;
	data->accel_y = sum_ay / times;
	data->accel_z = sum_az / times;
}

/* @brief: 获取陀螺仪原始数据平均值
   @param: data - 输出陀螺仪平均值数据
   @param: times - 采样次数
   @return: 无
 */
void ICM_GET_Average_Gyro_Raw_data(ICM42688_Gyro_Raw_Data_t* data, uint16_t times)
{
	ICM42688_Raw_Data_t raw_data;
	int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
	for(uint8_t i = 0; i < times; i++){
		ICM42688_ReadSensorRawData(&raw_data);
		sum_gx +=raw_data.gyro_x;
		sum_gy +=raw_data.gyro_y;
		sum_gz +=raw_data.gyro_z;
	}
	data->gyro_x = sum_gx / times;
	data->gyro_y = sum_gy / times;
	data->gyro_z = sum_gz / times;
}

/* @brief: 读取单个寄存器
   @param: reg_address - 寄存器地址
   @return: 寄存器值 (出错返回0xFF)
 */
uint8_t ICM42688_Read_Single_Reg(uint8_t reg_address)
{
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	err = HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), reg_address, I2C_MEMADD_SIZE_8BIT, &reg, 1, HAL_MAX_DELAY);
	if(err != HAL_OK)
	{
		printf("[ERROR] %s:%d: 出错啦！\r\n", __FILE__, __LINE__);
		return 0xFF;
	}
	printf("reg address is %x     (hex format) reg val is %x\r\n", reg_address ,reg);
	return reg;
}

/* @brief: 读取设备ID
   @return: WHO_AM_I寄存器值
 */
uint8_t ICM42688_Read_WhoAmI(void)
{
	uint8_t ID;
	HAL_StatusTypeDef err;
	err = HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), ICM42688_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &ID, 1, HAL_MAX_DELAY);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: 出错啦！\r\n", __FILE__, __LINE__);
	if(err == HAL_OK)
		printf("success\r\n");
	return ID;
	
}
//BANK 0 
//打开相关外设单元
/* @brief: 配置电源管理
   @return: 无
 */
void ICM_cfg_PWR(void)
{
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	reg |= ICM42688_PWR_TEMP_ON;
	reg |= ICM42688_PWR_IDLE_ON;
	reg |= ICM42688_PWR_GYRO_LN;
	reg |= ICM42688_PWR_ACCEL_LN;
	err = ICM42688_Write_8BIT(ICM42688_PWR_REG_PWR_MGMT0, reg);
	
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	return ;
}




//BANK 0
//设置下降沿/上升沿的速度
/* @brief: 配置I2C驱动能力
   @return: 无
 */
void ICM_cfg_DRIVE(void)
{
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	reg |= ICM42688_DRIVE_I2C_SLEW_2_6ns;
	err = ICM42688_Write_8BIT(ICM42688_REG_DRIVE_CONFIG, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	return ;
}


//BANK 0
//每次重置ICM42688
/* @brief: 设备配置（软件复位）
   @return: 无
 */
void ICM_cfg_DEVICE(void)
{
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	
	uint8_t reg = 1;
	HAL_StatusTypeDef err;
	err = ICM42688_Write_8BIT(ICM42688_REG_DEVICE_CONFIG, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	return ;
}

/* @brief: 计算陷波滤波器余弦参数
   @param: f_khz - 目标频率(kHz)
   @param: NF_COSWZ_SEL - 输出coswz模式选择器
   @return: 计算出的nf_coswz值
 */
static uint16_t calc_nf_coswz(float f_khz, uint8_t* NF_COSWZ_SEL)
{
    float COSWZ = cosf(2.0f * PI * f_khz / 32.0f);
    float nf_coswz_val = 0.0f;
		uint16_t res;
    if (fabsf(COSWZ) <= 0.875f)
    {
        *NF_COSWZ_SEL = 0;
        nf_coswz_val = roundf(COSWZ * 256.0f);
    }
    else
    {
        *NF_COSWZ_SEL = 1;
        if (COSWZ > 0.875f)
        {
            nf_coswz_val = roundf(8.0f * (1.0f - COSWZ) * 256.0f);
        }
        else if  (COSWZ < -0.875f)
        {
            nf_coswz_val = roundf(-8.0f * (1.0f + COSWZ) * 256.0f);
        }
    }
		res = (uint16_t)(nf_coswz_val+256.0f);		//把nf_coswz_val从-255~256的范围映射到0~511
		if(res > 511)
			res = 511;
    return res;
}


//BANK 1
//配置notch滤波器
/* @brief: 配置陷波滤波器
   @param: nff - 陷波滤波器配置
   @return: 无
 */
void ICM_cfg_Notchfilter(NFF_Config_st* nff)
{
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_1);
	
	//检查滤除频段的范围是否在1kHz到3kHz
	if(nff->gyro_x_FdesireKHz<1.0f || nff->gyro_x_FdesireKHz>3.0f)
		printf("[ERROR] %s:%d:  gyro_x_FdesireKHz range error! \r\n", __func__, __LINE__);
	if(nff->gyro_y_FdesireKHz<1.0f || nff->gyro_y_FdesireKHz>3.0f)
		printf("[ERROR] %s:%d:  gyro_x_FdesireKHz range error! \r\n", __func__, __LINE__);
	if(nff->gyro_z_FdesireKHz<1.0f || nff->gyro_z_FdesireKHz>3.0f)
		printf("[ERROR] %s:%d:  gyro_x_FdesireKHz range error! \r\n", __func__, __LINE__);
	
	
	//相关传入的gyro_x_FdesireKHz，gyro_y_FdesireKHz，gyro_x_FdesireKHz
	//计算相对应的coswz和相关的coswz_sel位
	uint16_t Xcoswz, Ycoswz, Zcoswz;
	Xcoswz = calc_nf_coswz(nff->gyro_x_FdesireKHz, &nff->gyro_x_coswz_sel);
	Ycoswz = calc_nf_coswz(nff->gyro_y_FdesireKHz, &nff->gyro_y_coswz_sel);
	Zcoswz = calc_nf_coswz(nff->gyro_z_FdesireKHz, &nff->gyro_z_coswz_sel);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	//Xcosz的低八位写入寄存器
	reg = (Xcoswz&0xFF);
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC6, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	//Ycosz的低八位写入寄存器
	reg = (Ycoswz&0xFF);
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC7, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	//Zcosz的低八位写入寄存器
	reg = (Zcoswz&0xFF);
	
	reg = 0;
	reg |= (Xcoswz&0x100)>>8;
	reg |= (Ycoswz&0x100)>>7;
	reg |= (Zcoswz&0x100)>>6;
	reg |= nff->gyro_x_coswz_sel<<3;
	reg |= nff->gyro_y_coswz_sel<<4;
	reg |= nff->gyro_z_coswz_sel<<5;
	
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC9, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	//设置带宽
	reg = ICM42688_GYRO_NF_BW_329HZ;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC10, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);

	//开启notch滤波器
	err = ICM42688_Read_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	reg &= 0xFE;//最低位置0
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	return ;
}

//BANK 1
/* @brief: 配置陀螺仪抗混叠滤波器
   @param: aff - AAF配置参数
   @return: 无
 */
static void ICM42688_cfg_AFFfilter_GYRO(AAF_Config_st* aff )
{
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_1);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	/***************GYRO_AFF_Configue********************/
	// GYRO_AAF_DELT 0x0C
	reg = 0x3F&aff->delt;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC3, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	
	//GYRO_AAF_DELTSQR
	//低地址写入0x0Dh ICM42688_REG_GYRO_CONFIG_STATIC4
	reg = 0xFF&aff->deltsqr;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC4, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	//高地址写入0x0Eh ICM42688_REG_GYRO_CONFIG_STATIC5
	//并且配置GYRO_AAF_BITSHIFT
	reg = (aff->deltsqr>>8)&0x0F;
	reg |= (aff->bitshift&0x0F)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC5, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	//开启gyro AFF滤波器
	err = ICM42688_Read_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	reg &= 0xFD;//次低位置0
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
}


//BANK 2
/* @brief: 配置加速度计抗混叠滤波器
   @param: aff - AAF配置参数
   @return: 无
 */
static void ICM42688_cfg_AFFfilter_ACCEL(AAF_Config_st* aff )
{
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_2);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	
	#if TEST_MODE
	printf("%d %d %d %d \r\n", aff->bw, aff->delt, aff->deltsqr, aff->bitshift);
	#endif
	
	/***************ACCEL_AFF_Configue********************/
	// ACCEL_AAF_DELT
	reg = aff->delt<<1;
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	// ACCEL_AAF_DELTSQR
	//低地址写入0x04h ICM42688_REG_ACCEL_CONFIG_STATIC3
	reg = 0xFF&aff->deltsqr;
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG_STATIC3, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! r\n", __func__, __LINE__);
	//高地址写入0x0Eh ICM42688_REG_GYRO_CONFIG_STATIC5
	//并且配置GYRO_AAF_BITSHIFT
	reg = (aff->deltsqr>>8)&0x0F;
	reg |= (aff->bitshift&0x0F)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG_STATIC4, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	//开启accel AFF滤波器
	err = ICM42688_Read_8BIT(ICM42688_REG_ACCEL_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	reg &= 0xFE;//最低位置0
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG_STATIC2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! r\n", __func__, __LINE__);
}


/* @brief: 配置加速度计和陀螺仪AAF
   @param: aff_accel - 加速度计AAF配置
   @param: aff_gyro - 陀螺仪AAF配置
   @return: 无
 */
void ICM_cfg_AFFfilter(AAF_Config_st* aff_accel, AAF_Config_st* aff_gyro)
{
	/***************ACCEL_AFF_Configue********************/
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_1);
	ICM42688_cfg_AFFfilter_GYRO(aff_gyro);
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_2);
	ICM42688_cfg_AFFfilter_ACCEL(aff_accel);
	
	//GYRO_AAF_DIS
	#if TEST_MODE
	printf("r\n");
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_1);
	ICM42688_Read_Single_Reg(ICM42688_REG_GYRO_CONFIG_STATIC3);
	ICM42688_Read_Single_Reg(ICM42688_REG_GYRO_CONFIG_STATIC4);
	ICM42688_Read_Single_Reg(ICM42688_REG_GYRO_CONFIG_STATIC5);
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_2);
	
	ICM42688_Read_Single_Reg(ICM42688_REG_ACCEL_CONFIG_STATIC2);
	ICM42688_Read_Single_Reg(ICM42688_REG_ACCEL_CONFIG_STATIC3);
	ICM42688_Read_Single_Reg(ICM42688_REG_ACCEL_CONFIG_STATIC4);
	#endif
}
//BANK 4
/* @brief: 写入可编程偏移值
   @param: POs - 偏移值结构体
   @return: 无
 */
void ICM_PROGRAMMABLE_OFFSET(programmable_Offset_st* POs)
{
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_4);
	
	uint8_t reg = 0;
	HAL_StatusTypeDef err;	
	
	reg = POs->X_gyro_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = (POs->X_gyro_offset>>8)&0xF;
	reg |= ((POs->Y_gyro_offset>>8)&0xF)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = POs->Y_gyro_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER2, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = POs->Z_gyro_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER3, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = (POs->Z_gyro_offset>>8)&0xF;
	reg |= ((POs->X_accel_offset>>8)&0xF)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER4, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = POs->X_accel_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER5, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = POs->Y_accel_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER6, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = (POs->Y_accel_offset>>8)&0xF;
	reg |= ((POs->Z_accel_offset>>8)&0xF)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER7, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	reg = POs->Z_accel_offset&0xFF;
	err = ICM42688_Write_8BIT(ICM42688_REG_OFFSET_USER8, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = 0;
	
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
}

/* @brief: 配置UI滤波器设置
   @param: UiFilter - 滤波器配置参数
   @return: 无
 */
void ICM_FILTER_Block(UI_FILTER_Block_st* UiFilter)
{
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	
	uint8_t reg = 0;
	
	HAL_StatusTypeDef err;
	reg |= ICM42688_GYRO_FS_2000DPS;
	reg |= ICM42688_GYRO_ODR_500HZ;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	
	reg = 0;
	err = ICM42688_Read_8BIT(ICM42688_REG_GYRO_CONFIG1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg &= 0xF5;		//把bit1 bit2 清0
	reg |= (UiFilter->GYRO_UI_FILT_ORD&0x03)<<1;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	reg = 0;				
	err = ICM42688_Read_8BIT(ICM42688_REG_ACCEL_CONFIG1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg &= 0xE7;		//把bit4 bit5 清0
	reg |= (UiFilter->ACCEL_UI_FILT_ORD&0x03)<<3;
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	
	
	reg = 0;
	reg |= (UiFilter->GYRO_UI_FILT_BW&0x0F);
	reg |= (UiFilter->ACCEL_UI_FILT_BW&0x0F)<<4;
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_ACCEL_CONFIG0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
}
//配置ICM的数据输出速率和输出范围
/* @brief: 设置输出数据率和满量程范围
   @param: OSs - ODR和FSR配置
   @return: 无
 */
void ICM_User_PATH_ODR_AND_FSR(ODR_SFR_st* OSs)
{
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	
	reg = OSs->GYRO_ODR&0xF;
	reg |= (OSs->GYRO_FSR<<5);
	err = ICM42688_Write_8BIT(ICM42688_REG_GYRO_CONFIG0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	reg = OSs->ACCEL_ODR&0xF;
	reg |= (OSs->ACCEL_FSR<<5);
	err = ICM42688_Write_8BIT(ICM42688_REG_ACCEL_CONFIG0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
}


/* @brief: 读取并打印温度数据
   @return: 无
 */
void ICM_GET_TEMP(void)
{
	int16_t temp_raw = 0;
	float temp = 0;
	uint8_t reg = 0;
	HAL_StatusTypeDef err;
	err = ICM42688_Read_8BIT(ICM42688_REG_TEMP_DATA1, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	temp_raw |= reg<<8;
	
	err = ICM42688_Read_8BIT(ICM42688_REG_TEMP_DATA0, reg);
	if(err != HAL_OK)
		printf("[ERROR] %s:%d: I2C error! \r\n", __func__, __LINE__);
	temp_raw |= reg;
	
	temp = (temp_raw/ 132.48f) + 25.0f;
	printf("temp is %f\r\n", temp);
}



/* @brief: 读取原始加速度计和陀螺仪数据
   @param: data - 输出传感器数据
   @return: HAL状态
 */
HAL_StatusTypeDef ICM42688_ReadSensorRawData(ICM42688_Raw_Data_t *data)
{
    uint8_t buf[12] = {0};
    HAL_StatusTypeDef ret;

    // 连续读取12字节：从 ACCEL_X1（0x66）开始，到 GYRO_Z0（0x71）
    ret = HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), ICM42688_REG_ACCEL_DATA_X1, I2C_MEMADD_SIZE_8BIT, buf, 12, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    // 将数据组合成 int16_t（高字节在前）
    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->gyro_x  = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_y  = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_z  = (int16_t)((buf[10] << 8) | buf[11]);

    return ret;
}
/* @brief: 读取原始加速度计数据
   @param: data - 输出加速度计数据
   @return: HAL状态
 */
HAL_StatusTypeDef ICM42688_ReadAccRawData(ICM42688_Acc_Raw_Data_t *data)
{
    uint8_t buf[6] = {0};
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), ICM42688_REG_ACCEL_DATA_X1, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }
    // 将数据组合成 int16_t（高字节在前）
    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    return ret;
}

/* @brief: 读取原始陀螺仪数据
   @param: data - 输出陀螺仪数据
   @return: HAL状态
 */
HAL_StatusTypeDef ICM42688_ReadGyroRawData(ICM42688_Gyro_Raw_Data_t *data)
{
    uint8_t buf[6] = {0};
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&ICM_I2C_HANDLE, (ICM42688_ADDRESS<<1), ICM42688_REG_GYRO_DATA_X1, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }
		
    // 将数据组合成 int16_t（高字节在前）
    data->gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->gyro_z = (int16_t)((buf[4] << 8) | buf[5]);
    return ret;
}



/* @brief: 校准传感器并写入偏移值
   @param: OSs - 包含校准所需的FSR设置
   @return: 无
 */
void ICM42688_CalibrateGyroAndWriteOffset(ODR_SFR_st* OSs)
{
    ICM42688_Raw_Data_t raw_data;
    programmable_Offset_st offset = {0};
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    const int sample_count = 500;  // set sample count
		
    // 获取加速度计分辨率（单位：g/LSB）
    switch (OSs->ACCEL_FSR) {
        case ICM42688_ACCEL_FS_2G:  accel_curr_resolution = 2.0f / 32768.0f; break;
        case ICM42688_ACCEL_FS_4G:  accel_curr_resolution = 4.0f / 32768.0f; break;
        case ICM42688_ACCEL_FS_8G:  accel_curr_resolution = 8.0f / 32768.0f; break;
        case ICM42688_ACCEL_FS_16G: accel_curr_resolution = 16.0f / 32768.0f; break;
    }

    // 获取陀螺仪分辨率（单位：dps/LSB）
    switch (OSs->GYRO_FSR) {
        case ICM42688_GYRO_FS_15_625DPS:  gyro_curr_resolution = 15.625f / 32768.0f; break;
        case ICM42688_GYRO_FS_31_25DPS:   gyro_curr_resolution = 31.25f  / 32768.0f; break;
        case ICM42688_GYRO_FS_62_5DPS:    gyro_curr_resolution = 62.5f   / 32768.0f; break;
        case ICM42688_GYRO_FS_125DPS:     gyro_curr_resolution = 125.0f  / 32768.0f; break;
        case ICM42688_GYRO_FS_250DPS:     gyro_curr_resolution = 250.0f  / 32768.0f; break;
        case ICM42688_GYRO_FS_500DPS:     gyro_curr_resolution = 500.0f  / 32768.0f; break;
        case ICM42688_GYRO_FS_1000DPS:    gyro_curr_resolution = 1000.0f / 32768.0f; break;
        case ICM42688_GYRO_FS_2000DPS:    gyro_curr_resolution = 2000.0f / 32768.0f; break;
    }
		#if TEST_MODE_2
		ICM42688_Raw_Data_t Ssr_data;
		int i= 0;
		printf("************************************\r\n");
		for(; i < 50; i++){
			ICM42688_ReadSensorRawData(&Ssr_data);
			printf("accelx %d\t accely %d\t accelz %d\t\r\n", Ssr_data.accel_x,Ssr_data.accel_y,Ssr_data.accel_z);
		}
		i = 0;
		for(; i < 50; i++){
			ICM42688_ReadSensorRawData(&Ssr_data);
			printf("gyrox %d\t gyroy %d\t gyroz %d\t\r\n", Ssr_data.gyro_x,Ssr_data.gyro_y,Ssr_data.gyro_z);
		}
		printf("************************************\r\n");
		printf("\r\n");
		printf("\r\n");
		printf("\r\n");
		printf("************************************\r\n");
    printf("[INFO] Start Gyro Offset Calibration...\r\n");
		
		#endif
		

    for (int i = 0; i < sample_count; i++) {
        if (ICM42688_ReadSensorRawData(&raw_data) != HAL_OK) {
            printf("[ERROR] Failed to read gyro data\r\n");
            return;
        }
#if 1				
				sum_gx +=raw_data.gyro_x;
				sum_gy +=raw_data.gyro_y;
				sum_gz +=raw_data.gyro_z;
				sum_ax +=raw_data.accel_x;
				sum_ay +=raw_data.accel_y;
				sum_az +=raw_data.accel_z;
#endif				
#if 0
        sum_gx += filterValue(&gx_filter, raw_data.gyro_x);
        sum_gy += filterValue(&gy_filter, raw_data.gyro_y);
        sum_gz += filterValue(&gz_filter, raw_data.gyro_z);
        sum_ax += filterValue(&ax_filter, raw_data.accel_x);
        sum_ay += filterValue(&ay_filter, raw_data.accel_y);
        sum_az += filterValue(&az_filter, raw_data.accel_z);
#endif
        HAL_Delay(10);  // 每次采样间隔
    }

    int16_t avg_gx = sum_gx / sample_count;
    int16_t avg_gy = sum_gy / sample_count;
    int16_t avg_gz = sum_gz / sample_count;
    int16_t avg_ax = sum_ax / sample_count;
    int16_t avg_ay = sum_ay / sample_count;
    int16_t avg_az = sum_az / sample_count;

#if TEST_MODE_2
    printf("Accel average bias: X=%d, Y=%d, Z=%d\r\n", avg_ax, avg_ay, avg_az);
    printf("Gyro  average bias: X=%d, Y=%d, Z=%d\r\n", avg_gx, avg_gy, avg_gz);
#endif

    // 加速度 offset（单位 g → offset LSB），Z轴考虑重力目标为 1g
    float az_g = accel_curr_resolution * avg_az;
    offset.X_accel_offset = -(int16_t)(accel_curr_resolution * avg_ax * 2048.0f);
    offset.Y_accel_offset = -(int16_t)(accel_curr_resolution * avg_ay * 2048.0f);
    offset.Z_accel_offset = -(int16_t)((az_g - 1.0f) / ACCEL_OFFSET_LSB_PER_G);

    // 陀螺仪 offset（单位 raw → offset LSB, 每 32 LSB = 1dps）
    offset.X_gyro_offset = -(int16_t)(avg_gx * gyro_curr_resolution * 32.0f);
    offset.Y_gyro_offset = -(int16_t)(avg_gy * gyro_curr_resolution * 32.0f);
    offset.Z_gyro_offset = -(int16_t)(avg_gz * gyro_curr_resolution * 32.0f);

#if TEST_MODE_2
    printf("Accel offset: X=%d, Y=%d, Z=%d\r\n", offset.X_accel_offset, offset.Y_accel_offset, offset.Z_accel_offset);
    printf("Gyro  offset: X=%d, Y=%d, Z=%d\r\n", offset.X_gyro_offset, offset.Y_gyro_offset, offset.Z_gyro_offset);
#endif

    // 限幅保护
    if (offset.X_gyro_offset > GYRO_OFFSET_MAX) offset.X_gyro_offset = GYRO_OFFSET_MAX;
    if (offset.X_gyro_offset < GYRO_OFFSET_MIN) offset.X_gyro_offset = GYRO_OFFSET_MIN;
    if (offset.Y_gyro_offset > GYRO_OFFSET_MAX) offset.Y_gyro_offset = GYRO_OFFSET_MAX;
    if (offset.Y_gyro_offset < GYRO_OFFSET_MIN) offset.Y_gyro_offset = GYRO_OFFSET_MIN;
    if (offset.Z_gyro_offset > GYRO_OFFSET_MAX) offset.Z_gyro_offset = GYRO_OFFSET_MAX;
    if (offset.Z_gyro_offset < GYRO_OFFSET_MIN) offset.Z_gyro_offset = GYRO_OFFSET_MIN;

    if (offset.X_accel_offset > ACCEL_OFFSET_MAX) offset.X_accel_offset = ACCEL_OFFSET_MAX;
    if (offset.X_accel_offset < ACCEL_OFFSET_MIN) offset.X_accel_offset = ACCEL_OFFSET_MIN;
    if (offset.Y_accel_offset > ACCEL_OFFSET_MAX) offset.Y_accel_offset = ACCEL_OFFSET_MAX;
    if (offset.Y_accel_offset < ACCEL_OFFSET_MIN) offset.Y_accel_offset = ACCEL_OFFSET_MIN;
    if (offset.Z_accel_offset > ACCEL_OFFSET_MAX) offset.Z_accel_offset = ACCEL_OFFSET_MAX;
    if (offset.Z_accel_offset < ACCEL_OFFSET_MIN) offset.Z_accel_offset = ACCEL_OFFSET_MIN;

    // 写入 offset 寄存器
    ICM_PROGRAMMABLE_OFFSET(&offset);

#if TEST_MODE_2
    printf("[INFO] Gyro Offset Calibration Done.\r\n");
		printf("************************************\r\n");
		printf("\r\n");
		printf("\r\n");
		printf("\r\n");
#endif
}

/* @brief: 初始化ICM42688传感器
   @return: 无
 */
void ICM42688_init(void)
{
	NFF_Config_st nff = {0};
	nff.gyro_x_FdesireKHz = 2.0f;
	nff.gyro_y_FdesireKHz = 2.0f;
	nff.gyro_z_FdesireKHz = 2.0f;
	nff.GYRO_NF_BW_SEL = ICM42688_GYRO_NF_BW_162HZ;
	//配置AAF参数
	AAF_Config_st aff_accel = ICM42688_AAF_ENTRY_2823;
	AAF_Config_st aff_gyro = ICM42688_AAF_ENTRY_2823;
	
	ODR_SFR_st OS = {0};
	//配置ODR和FSR (优化: 提高ODR和FSR以适应无人机高机动场景)
	OS.GYRO_ODR = ICM42688_GYRO_ODR_1KHZ;       // 1kHz采样率 (原500Hz)
	OS.GYRO_FSR = ICM42688_GYRO_FS_2000DPS;     // ±2000dps量程 (原±250dps)
	OS.ACCEL_ODR = ICM42688_ACCEL_ODR_1KHZ;     // 加速度计也提升到1kHz
	OS.ACCEL_FSR = ICM42688_ACCEL_FS_8G;         // ±8G (保持)
	
	//ICM_cfg_PWR();如果先执行ICM_cfg_PWR再执行软件复位就会把原来开启的温度，加速度计等置0
	ICM_cfg_DRIVE();//先进行软件复位
	ICM_cfg_DEVICE();
	
	HAL_Delay(30);
	ICM_cfg_PWR();
	
	ICM_cfg_Notchfilter(&nff);
	ICM_cfg_AFFfilter(&aff_accel,&aff_gyro);
	ICM_User_PATH_ODR_AND_FSR(&OS);
	
	ICM42688_CalibrateGyroAndWriteOffset(&OS);
	
	//最后切换成bank0
	ICM_BANK_SWITCH(ICM42688_BANK_SEL_0);
	
}
