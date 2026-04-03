#include "QMC5883P.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

#define QMC_I2C_HANDLE 		hi2c1
#define QMC_I2C_DelayMS		HAL_MAX_DELAY		

#define QMC_Write_8BIT(reg_address,wirte8bit)\
				HAL_I2C_Mem_Write(&QMC_I2C_HANDLE, (QMC5883P_ADDRESS<<1), (reg_address), I2C_MEMADD_SIZE_8BIT, &(wirte8bit), 1, QMC_I2C_DelayMS)

#define QMC_Read_8BIT(reg_address, read8bit)\
				HAL_I2C_Mem_Read(&QMC_I2C_HANDLE, (QMC5883P_ADDRESS<<1), (reg_address), I2C_MEMADD_SIZE_8BIT, &(read8bit), 1, QMC_I2C_DelayMS)

#define QMC_Read_6Bytes(reg_address, read8bit)\
				HAL_I2C_Mem_Read(&QMC_I2C_HANDLE, (QMC5883P_ADDRESS<<1), (reg_address), I2C_MEMADD_SIZE_8BIT, &(read8bit), 6, QMC_I2C_DelayMS)


#define resolution 0.000244140625			//8guass

// ========== 磁力计校准全局变量 ==========
static QMC5883P_Calibration_t s_calibration = {0, 0, 0, 0};
static QMC5883P_Full_Calibration_t s_full_calibration = {
    {0.0f, 0.0f, 0.0f},
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    },
    0,
    0
};


QMC_StatusTypeDef QMC_ReadChipID(uint8_t* ChipID)
{
	HAL_StatusTypeDef err;
	err = QMC_Read_8BIT(QMC5883P_REG_CHIPID_CONFIG, *ChipID);
	if(err != HAL_OK)	return QMC_ERROR;
	return QMC_OK;
}

QMC_StatusTypeDef QMC_Set_Ctrl1(void)//控制寄存器1?
{
	HAL_StatusTypeDef err;
	uint8_t reg = 0;
	//firstly, you should switch mode in "suspend mode"
	err = QMC_Write_8BIT(QMC5883P_REG_Ctrl1, reg);
	if(err != HAL_OK)	return QMC_ERROR;

	// 优化配置: 根据QMC5883数据手册推荐
	reg |= 0x01;      // normal mode
	reg |= 0x00<<2;   // ODR is 100Hz (原200Hz，降低功耗提高稳定性)
	reg |= 0x03<<4;   // OSR1 is 16 (原4，高OSR降低噪声)
	reg |= 0x03<<6;   // OSR2 is 64 (原2，配合OSR1实现OSR=1024)

	err = QMC_Write_8BIT(QMC5883P_REG_Ctrl1, reg);
	if(err != HAL_OK)	return QMC_ERROR;
	return QMC_OK;
}

QMC_StatusTypeDef QMC_Set_Ctrl2(void)//控制寄存器2？
{
	HAL_StatusTypeDef err;
	uint8_t reg = 0;
	
	
	reg |= 0x00;		//mode : set and reset on 
	reg |= 0x02<<2;	//Range : 8Guass
	
	err = QMC_Write_8BIT(QMC5883P_REG_Ctrl2, reg);
	if(err != HAL_OK)	return QMC_ERROR;
	return QMC_OK;
}

QMC_StatusTypeDef QMC_Reset(void)
{
	HAL_StatusTypeDef err;
	uint8_t reg = 0;
	reg |= 0x1<<7;		//mode : set and reset on 
	err = QMC_Write_8BIT(QMC5883P_REG_Ctrl2, reg);
	if(err != HAL_OK)	return QMC_ERROR;
	return QMC_OK;
}

QMC_StatusTypeDef QMC_Init(void)
{
	QMC_Reset();
	HAL_Delay(70);
	QMC_Set_Ctrl2();
	QMC_Set_Ctrl1();
	return QMC_OK;
}

void QMC_Read_REG_DATA(uint8_t REG_ADDR, uint8_t *data)
{
	HAL_StatusTypeDef err;
	err = QMC_Read_8BIT(REG_ADDR, *data);
	if(err != HAL_OK) printf("wrong\r\n");
}

QMC_StatusTypeDef QMC_Read_Raw_DATA(QMC5883P_Raw_Data_t* data)
{
	HAL_StatusTypeDef err;
	uint8_t buf[6];
	err = HAL_I2C_Mem_Read(&QMC_I2C_HANDLE, (QMC5883P_ADDRESS<<1), (QMC5883P_REG_XLSB), I2C_MEMADD_SIZE_8BIT, buf, 6, QMC_I2C_DelayMS);
	if(err != HAL_OK)	return QMC_ERROR;

	data->X = (int16_t)((buf[1] << 8) | buf[0]);  // X_MSB << 8 | X_LSB
	data->Y = (int16_t)((buf[3] << 8) | buf[2]);  // Y_MSB << 8 | Y_LSB
	data->Z = (int16_t)((buf[5] << 8) | buf[4]);  // Z_MSB << 8 | Z_LSB
	return QMC_OK;
}

QMC_StatusTypeDef QMC_Read_DATA(QMC5883P_Data_t* data)
{
	HAL_StatusTypeDef err;
	uint8_t buf[6];
	QMC5883P_Raw_Data_t Raw_data;
	err = HAL_I2C_Mem_Read(&QMC_I2C_HANDLE, (QMC5883P_ADDRESS<<1), (QMC5883P_REG_XLSB), I2C_MEMADD_SIZE_8BIT, buf, 6, QMC_I2C_DelayMS);
	if(err != HAL_OK)	return QMC_ERROR;

	Raw_data.X = (int16_t)((buf[1] << 8) | buf[0]);  // X_MSB << 8 | X_LSB
	Raw_data.Y = (int16_t)((buf[3] << 8) | buf[2]);  // Y_MSB << 8 | Y_LSB
	Raw_data.Z= (int16_t)((buf[5] << 8) | buf[4]);  // Z_MSB << 8 | Z_LSB
	
	data->X_Guass = Raw_data.X*resolution;
	data->Y_Guass = Raw_data.Y*resolution;
	data->Z_Guass = Raw_data.Z*resolution;
	return QMC_OK;
}

// ========== 磁力计校准功能实现 ==========

/**
 * @brief 执行磁力计校准（硬铁校准 - Hard Iron Calibration）
 * @param duration_seconds 校准持续时间（秒）
 * 
 * 校准原理：
 * 磁力计在旋转时，理想情况下应该在三维空间中形成一个以原点为中心的球体。
 * 但由于硬铁偏移（飞机上的永磁材料），球心会偏移。
 * 通过记录最大/最小值，计算偏移量并补偿。
 */
void QMC_Calibrate(uint32_t duration_seconds)
{
    QMC5883P_Raw_Data_t raw;
    int16_t max_x = -32768, max_y = -32768, max_z = -32768;
    int16_t min_x = 32767, min_y = 32767, min_z = 32767;
    
    printf("\r\n");
    printf("╔════════════════════════════════════════════════╗\r\n");
    printf("║        磁力计校准程序（硬铁校准）             ║\r\n");
    printf("╚════════════════════════════════════════════════╝\r\n\r\n");
    
    printf("校准时间：%lu 秒\r\n\r\n", duration_seconds);
    
    printf("操作方法：\r\n");
    printf("  1. 远离磁干扰源（电脑、手机、金属物体）\r\n");
    printf("  2. 在接下来的 %lu 秒内：\r\n", duration_seconds);
    printf("     - 缓慢旋转飞机，覆盖所有方向\r\n");
    printf("     - 倾斜飞机，朝各个角度转动\r\n");
    printf("     - 做8字形运动效果最好\r\n");
    printf("  3. 尽量让飞机在空间中经过所有姿态\r\n\r\n");
    
    printf("准备开始校准...\r\n");
    printf("5秒倒计时...\r\n");
    
    for(int i = 5; i > 0; i--) {
        printf("%d... ", i);
        HAL_Delay(1000);
    }
    
    printf("\r\n\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("开始校准！请旋转飞机...\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t sample_count = 0;
    uint32_t last_print_time = start_time;
    
    // 采集数据
    while((HAL_GetTick() - start_time) < (duration_seconds * 1000)) {
        if(QMC_Read_Raw_DATA(&raw) == QMC_OK) {
            // 更新最大值
            if(raw.X > max_x) max_x = raw.X;
            if(raw.Y > max_y) max_y = raw.Y;
            if(raw.Z > max_z) max_z = raw.Z;
            
            // 更新最小值
            if(raw.X < min_x) min_x = raw.X;
            if(raw.Y < min_y) min_y = raw.Y;
            if(raw.Z < min_z) min_z = raw.Z;
            
            sample_count++;
            
            // 每秒打印一次进度
            uint32_t current_time = HAL_GetTick();
            if((current_time - last_print_time) >= 1000) {
                uint32_t elapsed = (current_time - start_time) / 1000;
                
                printf("进度: %lu/%lu 秒  |  样本数: %lu  |  ", 
                       elapsed, duration_seconds, sample_count);
                printf("X:[%d,%d] Y:[%d,%d] Z:[%d,%d]\r\n",
                       min_x, max_x, min_y, max_y, min_z, max_z);
                
                last_print_time = current_time;
            }
        }
        
        HAL_Delay(10);  // 100Hz采样率
    }
    
    printf("\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("校准完成！\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
    
    // 计算偏移量（硬铁偏移 = (最大值 + 最小值) / 2）
    s_calibration.offset_x = (max_x + min_x) / 2;
    s_calibration.offset_y = (max_y + min_y) / 2;
    s_calibration.offset_z = (max_z + min_z) / 2;
    s_calibration.is_calibrated = 1;
    
    // 打印校准结果
    printf("校准结果：\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("采集样本数：%lu\r\n\r\n", sample_count);
    
    printf("原始数据范围：\r\n");
    printf("  X轴: [%6d, %6d]  范围: %d\r\n", min_x, max_x, max_x - min_x);
    printf("  Y轴: [%6d, %6d]  范围: %d\r\n", min_y, max_y, max_y - min_y);
    printf("  Z轴: [%6d, %6d]  范围: %d\r\n\r\n", min_z, max_z, max_z - min_z);
    
    printf("计算的硬铁偏移：\r\n");
    printf("  offset_x = %d\r\n", s_calibration.offset_x);
    printf("  offset_y = %d\r\n", s_calibration.offset_y);
    printf("  offset_z = %d\r\n\r\n", s_calibration.offset_z);
    
    // 计算校准后的范围
    int16_t cal_range_x = max_x - min_x;
    int16_t cal_range_y = max_y - min_y;
    int16_t cal_range_z = max_z - min_z;
    
    printf("校准后数据范围（应该相近）：\r\n");
    printf("  X轴范围: %d  (±%d)\r\n", cal_range_x, cal_range_x/2);
    printf("  Y轴范围: %d  (±%d)\r\n", cal_range_y, cal_range_y/2);
    printf("  Z轴范围: %d  (±%d)\r\n\r\n", cal_range_z, cal_range_z/2);
    
    // 质量检查
    int16_t avg_range = (cal_range_x + cal_range_y + cal_range_z) / 3;
    int16_t max_deviation = 0;
    
    if(abs(cal_range_x - avg_range) > max_deviation) 
        max_deviation = abs(cal_range_x - avg_range);
    if(abs(cal_range_y - avg_range) > max_deviation) 
        max_deviation = abs(cal_range_y - avg_range);
    if(abs(cal_range_z - avg_range) > max_deviation) 
        max_deviation = abs(cal_range_z - avg_range);
    
    float deviation_percent = (float)max_deviation / avg_range * 100.0f;
    
    printf("校准质量评估：\r\n");
    if(deviation_percent < 10.0f) {
        printf("  [OK] 优秀！偏差 < 10%% (%.1f%%)\r\n", deviation_percent);
    } else if(deviation_percent < 20.0f) {
        printf("  [OK] 良好！偏差 < 20%% (%.1f%%)\r\n", deviation_percent);
    } else if(deviation_percent < 30.0f) {
        printf("  [!] 一般，偏差 < 30%% (%.1f%%)\r\n", deviation_percent);
        printf("  建议重新校准，覆盖更多方向\r\n");
    } else {
        printf("  [X] 较差，偏差 %.1f%%\r\n", deviation_percent);
        printf("  建议：\r\n");
        printf("    1. 远离磁干扰\r\n");
        printf("    2. 更充分地旋转飞机\r\n");
        printf("    3. 重新校准\r\n");
    }
    
    printf("\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("校准数据已保存到内存\r\n");
    printf("后续读取将自动应用校准\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
}

/**
 * @brief 获取当前校准数据
 */
void QMC_Get_Calibration(QMC5883P_Calibration_t* calib)
{
    if(calib) {
        calib->offset_x = s_calibration.offset_x;
        calib->offset_y = s_calibration.offset_y;
        calib->offset_z = s_calibration.offset_z;
        calib->is_calibrated = s_calibration.is_calibrated;
    }
}

/**
 * @brief 设置校准数据（用于加载保存的校准值）
 */
void QMC_Set_Calibration(const QMC5883P_Calibration_t* calib)
{
    if(calib) {
        s_calibration.offset_x = calib->offset_x;
        s_calibration.offset_y = calib->offset_y;
        s_calibration.offset_z = calib->offset_z;
        s_calibration.is_calibrated = calib->is_calibrated;
        
        printf("[QMC5883P] 校准数据已加载：\r\n");
        printf("  offset_x = %d\r\n", s_calibration.offset_x);
        printf("  offset_y = %d\r\n", s_calibration.offset_y);
        printf("  offset_z = %d\r\n", s_calibration.offset_z);
    }
}

/**
 * @brief 读取校准后的原始数据（已应用偏移）
 */
QMC_StatusTypeDef QMC_Read_Calibrated_Raw_DATA(QMC5883P_Raw_Data_t* data)
{
    QMC_StatusTypeDef status = QMC_Read_Raw_DATA(data);
    
    if(status == QMC_OK && s_calibration.is_calibrated) {
        // 应用硬铁偏移校准
        data->X -= s_calibration.offset_x;
        data->Y -= s_calibration.offset_y;
        data->Z -= s_calibration.offset_z;
    }
    
    return status;
}

/**
 * @brief 重置校准数据
 */
void QMC_Reset_Calibration(void)
{
    s_calibration.offset_x = 0;
    s_calibration.offset_y = 0;
    s_calibration.offset_z = 0;
    s_calibration.is_calibrated = 0;
    
    printf("[QMC5883P] 校准数据已重置\r\n");
}

/**
 * @brief 打印当前校准状态
 */
void QMC_Print_Calibration_Info(void)
{
    printf("\r\n");
    printf("╔════════════════════════════════════════════════╗\r\n");
    printf("║        磁力计校准状态                         ║\r\n");
    printf("╚════════════════════════════════════════════════╝\r\n\r\n");
    
    if(s_calibration.is_calibrated) {
        printf("状态：[OK] 已校准\r\n\r\n");
        printf("硬铁偏移值：\r\n");
        printf("  offset_x = %d\r\n", s_calibration.offset_x);
        printf("  offset_y = %d\r\n", s_calibration.offset_y);
        printf("  offset_z = %d\r\n\r\n", s_calibration.offset_z);
        
        // 读取并显示校准前后对比
        QMC5883P_Raw_Data_t raw_uncal, raw_cal;
        if(QMC_Read_Raw_DATA(&raw_uncal) == QMC_OK) {
            raw_cal.X = raw_uncal.X - s_calibration.offset_x;
            raw_cal.Y = raw_uncal.Y - s_calibration.offset_y;
            raw_cal.Z = raw_uncal.Z - s_calibration.offset_z;
            
            printf("当前读数对比：\r\n");
            printf("             原始值    校准后\r\n");
            printf("  X轴：    %6d    %6d\r\n", raw_uncal.X, raw_cal.X);
            printf("  Y轴：    %6d    %6d\r\n", raw_uncal.Y, raw_cal.Y);
            printf("  Z轴：    %6d    %6d\r\n", raw_uncal.Z, raw_cal.Z);
        }
    } else {
        printf("状态：[X] 未校准\r\n\r\n");
        printf("建议：\r\n");
        printf("  1. 调用 QMC_Calibrate(30) 进行校准\r\n");
        printf("  2. 校准时间建议 30-60 秒\r\n");
        printf("  3. 校准时需要充分旋转飞机\r\n");
    }
    
    printf("\r\n");
}

// ========== 软铁校准功能实现 ==========

/**
 * @brief 执行完整校准(硬铁+软铁)
 * @param duration_seconds 校准持续时间(秒),建议60秒
 * 
 * 校准原理:
 * 1. 硬铁校准: 计算椭球中心偏移(永磁材料影响)
 * 2. 软铁校准: 计算椭球各轴半径(软磁材料影响,导致磁场畸变)
 */
void QMC_Calibrate_Full(uint32_t duration_seconds)
{
    QMC5883P_Raw_Data_t raw;
    int16_t max_x = -32768, max_y = -32768, max_z = -32768;
    int16_t min_x = 32767, min_y = 32767, min_z = 32767;
    
    printf("\r\n");
    printf("╔═══════════════════════════════════════════════════╗\r\n");
    printf("║     磁力计完整校准（硬铁+软铁）                  ║\r\n");
    printf("╚═══════════════════════════════════════════════════╝\r\n\r\n");
    
    printf("校准时间：%lu 秒\r\n\r\n", duration_seconds);
    
    printf("操作方法：\r\n");
    printf("  1. 远离磁干扰源（电脑、手机、金属物体）\r\n");
    printf("  2. 在接下来的 %lu 秒内：\r\n", duration_seconds);
    printf("     - 缓慢旋转飞机，覆盖所有方向\r\n");
    printf("     - 倾斜飞机，朝各个角度转动\r\n");
    printf("     - 做8字形运动效果最好\r\n");
    printf("  3. 尽量让飞机在空间中经过所有姿态\r\n\r\n");
    
    printf("准备开始校准...\r\n");
    printf("3秒倒计时...\r\n");
    
    for(int i = 3; i > 0; i--) {
        printf("%d... ", i);
        HAL_Delay(1000);
    }
    
    printf("\r\n\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("开始校准！请旋转飞机...\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t sample_count = 0;
    uint32_t last_print_time = start_time;
    
    // 采集数据
    while((HAL_GetTick() - start_time) < (duration_seconds * 1000)) {
        if(QMC_Read_Raw_DATA(&raw) == QMC_OK) {
            // 更新最大值
            if(raw.X > max_x) max_x = raw.X;
            if(raw.Y > max_y) max_y = raw.Y;
            if(raw.Z > max_z) max_z = raw.Z;
            
            // 更新最小值
            if(raw.X < min_x) min_x = raw.X;
            if(raw.Y < min_y) min_y = raw.Y;
            if(raw.Z < min_z) min_z = raw.Z;
            
            sample_count++;
            
            // 每秒打印一次进度
            uint32_t current_time = HAL_GetTick();
            if((current_time - last_print_time) >= 1000) {
                uint32_t elapsed = (current_time - start_time) / 1000;
                
                printf("进度: %lu/%lu 秒  |  样本数: %lu  |  ", 
                       elapsed, duration_seconds, sample_count);
                printf("X:[%d,%d] Y:[%d,%d] Z:[%d,%d]\r\n",
                       min_x, max_x, min_y, max_y, min_z, max_z);
                
                last_print_time = current_time;
            }
        }
        
        HAL_Delay(10);  // 100Hz采样率
    }
    
    printf("\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("校准完成！\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
    
    // ========== 步骤1：计算硬铁偏移（椭球中心）==========
    s_full_calibration.offset[0] = (float)(max_x + min_x) / 2.0f;
    s_full_calibration.offset[1] = (float)(max_y + min_y) / 2.0f;
    s_full_calibration.offset[2] = (float)(max_z + min_z) / 2.0f;
    s_full_calibration.is_hard_iron_calibrated = 1;
    
    // ========== 步骤2：计算软铁缩放因子（椭球半轴） ==========
    int16_t range_x = max_x - min_x;
    int16_t range_y = max_y - min_y;
    int16_t range_z = max_z - min_z;
    
    // 计算平均半径（目标球体半径）
    float avg_radius = (range_x + range_y + range_z) / 6.0f;  // 除6因为范围=2*半径
    
    // 计算各轴缩放因子（将椭球变换为球体）
    s_full_calibration.soft_matrix[0][0] = (range_x > 0) ? (2.0f * avg_radius / range_x) : 1.0f;
    s_full_calibration.soft_matrix[1][1] = (range_y > 0) ? (2.0f * avg_radius / range_y) : 1.0f;
    s_full_calibration.soft_matrix[2][2] = (range_z > 0) ? (2.0f * avg_radius / range_z) : 1.0f;
    s_full_calibration.soft_matrix[0][1] = 0.0f;
    s_full_calibration.soft_matrix[0][2] = 0.0f;
    s_full_calibration.soft_matrix[1][0] = 0.0f;
    s_full_calibration.soft_matrix[1][2] = 0.0f;
    s_full_calibration.soft_matrix[2][0] = 0.0f;
    s_full_calibration.soft_matrix[2][1] = 0.0f;
    s_full_calibration.is_soft_iron_calibrated = 1;
    
    // ========== 步骤3：打印校准结果 ==========
    printf("校准结果:\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("采集样本数: %lu\r\n\r\n", sample_count);
    
    printf("[硬铁校准] 偏移值(椭球中心):\r\n");
    printf("  offset_x = %.2f\r\n", s_full_calibration.offset[0]);
    printf("  offset_y = %.2f\r\n", s_full_calibration.offset[1]);
    printf("  offset_z = %.2f\r\n\r\n", s_full_calibration.offset[2]);
    
    printf("[软铁校准] 缩放因子(椭球->球体):\r\n");
    printf("  m00 = %.4f\r\n", s_full_calibration.soft_matrix[0][0]);
    printf("  m11 = %.4f\r\n", s_full_calibration.soft_matrix[1][1]);
    printf("  m22 = %.4f\r\n\r\n", s_full_calibration.soft_matrix[2][2]);
    
    printf("原始数据范围:\r\n");
    printf("  X轴: [%6d, %6d]  范围: %d\r\n", min_x, max_x, range_x);
    printf("  Y轴: [%6d, %6d]  范围: %d\r\n", min_y, max_y, range_y);
    printf("  Z轴: [%6d, %6d]  范围: %d\r\n\r\n", min_z, max_z, range_z);
    
    // ========== 步骤4：质量评估 ==========
    float max_scale = s_full_calibration.soft_matrix[0][0];
    float min_scale = s_full_calibration.soft_matrix[0][0];
    
    if(s_full_calibration.soft_matrix[1][1] > max_scale) max_scale = s_full_calibration.soft_matrix[1][1];
    if(s_full_calibration.soft_matrix[2][2] > max_scale) max_scale = s_full_calibration.soft_matrix[2][2];
    
    if(s_full_calibration.soft_matrix[1][1] < min_scale) min_scale = s_full_calibration.soft_matrix[1][1];
    if(s_full_calibration.soft_matrix[2][2] < min_scale) min_scale = s_full_calibration.soft_matrix[2][2];
    
    float scale_deviation = (max_scale - min_scale) / ((max_scale + min_scale) / 2.0f) * 100.0f;
    
    printf("校准质量评估:\r\n");
    printf("  平均半径: %.1f counts\r\n", avg_radius);
    printf("  缩放因子偏差: %.1f%%\r\n", scale_deviation);
    
    if(scale_deviation < 10.0f) {
        printf("  [OK] 优秀！软铁畸变 < 10%%\r\n");
    } else if(scale_deviation < 20.0f) {
        printf("  [OK] 良好！软铁畸变 < 20%%\r\n");
    } else if(scale_deviation < 30.0f) {
        printf("  [!] 一般，软铁畸变 < 30%%\r\n");
        printf("  建议：检查飞机周围是否有软磁材料\r\n");
    } else {
        printf("  [X] 较差，软铁畸变 %.1f%%\r\n", scale_deviation);
        printf("  建议：\r\n");
        printf("    1. 重新安装磁力计，远离电机和电池\r\n");
        printf("    2. 更充分地旋转飞机重新校准\r\n");
    }
    
    printf("\r\n");
    printf("保存以下代码到程序：\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n");
    printf("QMC5883P_Full_Calibration_t calib = {\r\n");
    printf("    {%.2ff, %.2ff, %.2ff},\r\n",
           s_full_calibration.offset[0],
           s_full_calibration.offset[1],
           s_full_calibration.offset[2]);
    printf("    {\r\n");
    printf("        {%.4ff, %.4ff, %.4ff},\r\n",
           s_full_calibration.soft_matrix[0][0],
           s_full_calibration.soft_matrix[0][1],
           s_full_calibration.soft_matrix[0][2]);
    printf("        {%.4ff, %.4ff, %.4ff},\r\n",
           s_full_calibration.soft_matrix[1][0],
           s_full_calibration.soft_matrix[1][1],
           s_full_calibration.soft_matrix[1][2]);
    printf("        {%.4ff, %.4ff, %.4ff}\r\n",
           s_full_calibration.soft_matrix[2][0],
           s_full_calibration.soft_matrix[2][1],
           s_full_calibration.soft_matrix[2][2]);
    printf("    },\r\n");
    printf("    1,\r\n");
    printf("    1\r\n");
    printf("};\r\n");
    printf("QMC_Set_Full_Calibration(&calib);\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
}

/**
 * @brief 获取完整校准数据
 */
void QMC_Get_Full_Calibration(QMC5883P_Full_Calibration_t* calib)
{
    if(calib) {
        int row;
        int col;
        
        calib->offset[0] = s_full_calibration.offset[0];
        calib->offset[1] = s_full_calibration.offset[1];
        calib->offset[2] = s_full_calibration.offset[2];
        
        for(row = 0; row < 3; row++) {
            for(col = 0; col < 3; col++) {
                calib->soft_matrix[row][col] = s_full_calibration.soft_matrix[row][col];
            }
        }
        calib->is_hard_iron_calibrated = s_full_calibration.is_hard_iron_calibrated;
        calib->is_soft_iron_calibrated = s_full_calibration.is_soft_iron_calibrated;
    }
}

/**
 * @brief 设置完整校准数据
 */
void QMC_Set_Full_Calibration(const QMC5883P_Full_Calibration_t* calib)
{
    if(calib) {
        int row;
        int col;
        
        s_full_calibration.offset[0] = calib->offset[0];
        s_full_calibration.offset[1] = calib->offset[1];
        s_full_calibration.offset[2] = calib->offset[2];
        
        for(row = 0; row < 3; row++) {
            for(col = 0; col < 3; col++) {
                s_full_calibration.soft_matrix[row][col] = calib->soft_matrix[row][col];
            }
        }
        s_full_calibration.is_hard_iron_calibrated = calib->is_hard_iron_calibrated;
        s_full_calibration.is_soft_iron_calibrated = calib->is_soft_iron_calibrated;
        
        printf("[QMC5883P] 完整校准数据已加载：\r\n");
        printf("  硬铁偏移: [%.2f, %.2f, %.2f]\r\n", 
               s_full_calibration.offset[0], s_full_calibration.offset[1], s_full_calibration.offset[2]);
        printf("  软铁矩阵:\r\n");
        for(row = 0; row < 3; row++) {
            printf("    [%.4f, %.4f, %.4f]\r\n",
                   s_full_calibration.soft_matrix[row][0],
                   s_full_calibration.soft_matrix[row][1],
                   s_full_calibration.soft_matrix[row][2]);
        }
    }
}

/**
 * @brief 读取完整校准后的数据（硬铁+软铁）
 */
QMC_StatusTypeDef QMC_Read_Full_Calibrated_DATA(QMC5883P_Raw_Data_t* data)
{
    QMC_StatusTypeDef status = QMC_Read_Raw_DATA(data);
    float x;
    float y;
    float z;
    float cal_x;
    float cal_y;
    float cal_z;
    
    if(status == QMC_OK) {
        x = (float)data->X;
        y = (float)data->Y;
        z = (float)data->Z;
        
        if(s_full_calibration.is_hard_iron_calibrated) {
            x -= s_full_calibration.offset[0];
            y -= s_full_calibration.offset[1];
            z -= s_full_calibration.offset[2];
        }
        
        if(s_full_calibration.is_soft_iron_calibrated) {
            cal_x = s_full_calibration.soft_matrix[0][0] * x +
                    s_full_calibration.soft_matrix[0][1] * y +
                    s_full_calibration.soft_matrix[0][2] * z;
            cal_y = s_full_calibration.soft_matrix[1][0] * x +
                    s_full_calibration.soft_matrix[1][1] * y +
                    s_full_calibration.soft_matrix[1][2] * z;
            cal_z = s_full_calibration.soft_matrix[2][0] * x +
                    s_full_calibration.soft_matrix[2][1] * y +
                    s_full_calibration.soft_matrix[2][2] * z;
        } else {
            cal_x = x;
            cal_y = y;
            cal_z = z;
        }
        
        if(cal_x > 32767.0f) cal_x = 32767.0f;
        if(cal_x < -32768.0f) cal_x = -32768.0f;
        if(cal_y > 32767.0f) cal_y = 32767.0f;
        if(cal_y < -32768.0f) cal_y = -32768.0f;
        if(cal_z > 32767.0f) cal_z = 32767.0f;
        if(cal_z < -32768.0f) cal_z = -32768.0f;
        
        data->X = (int16_t)cal_x;
        data->Y = (int16_t)cal_y;
        data->Z = (int16_t)cal_z;
    }
    
    return status;
}

/**
 * @brief 打印完整校准信息
 */
void QMC_Print_Full_Calibration_Info(void)
{
    // 在函数开头声明所有局部变量（C90标准要求）
    QMC5883P_Raw_Data_t raw, calibrated;
    float raw_mag, cal_mag;
    float avg_scale, max_dev;
    float dev_x, dev_y, dev_z;
    
    printf("\r\n");
    printf("===============================================\r\n");
    printf("        磁力计完整校准状态\r\n");
    printf("===============================================\r\n\r\n");
    
    printf("硬铁校准状态: ");
    if(s_full_calibration.is_hard_iron_calibrated) {
        printf("[OK] 已校准\r\n");
    } else {
        printf("[X] 未校准\r\n");
    }
    
    printf("软铁校准状态: ");
    if(s_full_calibration.is_soft_iron_calibrated) {
        printf("[OK] 已校准\r\n\r\n");
    } else {
        printf("[X] 未校准\r\n\r\n");
    }
    
    if(s_full_calibration.is_hard_iron_calibrated) {
        printf("[硬铁偏移]\r\n");
        printf("  offset_x = %.2f\r\n", s_full_calibration.offset[0]);
        printf("  offset_y = %.2f\r\n", s_full_calibration.offset[1]);
        printf("  offset_z = %.2f\r\n\r\n", s_full_calibration.offset[2]);
    }
    
    if(s_full_calibration.is_soft_iron_calibrated) {
        printf("[软铁矩阵]\r\n");
        printf("  | %.4f  %.4f  %.4f |\r\n",
               s_full_calibration.soft_matrix[0][0],
               s_full_calibration.soft_matrix[0][1],
               s_full_calibration.soft_matrix[0][2]);
        printf("  | %.4f  %.4f  %.4f |\r\n",
               s_full_calibration.soft_matrix[1][0],
               s_full_calibration.soft_matrix[1][1],
               s_full_calibration.soft_matrix[1][2]);
        printf("  | %.4f  %.4f  %.4f |\r\n\r\n",
               s_full_calibration.soft_matrix[2][0],
               s_full_calibration.soft_matrix[2][1],
               s_full_calibration.soft_matrix[2][2]);
        
        avg_scale = (s_full_calibration.soft_matrix[0][0] +
                     s_full_calibration.soft_matrix[1][1] +
                     s_full_calibration.soft_matrix[2][2]) / 3.0f;
        max_dev = 0.0f;
        
        dev_x = fabsf(s_full_calibration.soft_matrix[0][0] - avg_scale) / avg_scale * 100.0f;
        dev_y = fabsf(s_full_calibration.soft_matrix[1][1] - avg_scale) / avg_scale * 100.0f;
        dev_z = fabsf(s_full_calibration.soft_matrix[2][2] - avg_scale) / avg_scale * 100.0f;
        
        if(dev_x > max_dev) max_dev = dev_x;
        if(dev_y > max_dev) max_dev = dev_y;
        if(dev_z > max_dev) max_dev = dev_z;
        
        printf("  最大畸变: %.1f%%", max_dev);
        if(max_dev < 10.0f) {
            printf(" [OK]\r\n");
        } else if(max_dev < 20.0f) {
            printf(" [OK]\r\n");
        } else {
            printf(" [!] 建议检查磁力计安装位置\r\n");
        }
    }
    
    // 显示校准前后对比
    if(s_full_calibration.is_hard_iron_calibrated || s_full_calibration.is_soft_iron_calibrated) {
        if(QMC_Read_Raw_DATA(&raw) == QMC_OK) {
            QMC_Read_Full_Calibrated_DATA(&calibrated);
            
            raw_mag = sqrtf(raw.X * raw.X + raw.Y * raw.Y + raw.Z * raw.Z) * resolution;
            cal_mag = sqrtf(calibrated.X * calibrated.X + 
                           calibrated.Y * calibrated.Y + 
                           calibrated.Z * calibrated.Z) * resolution;
            
            printf("\r\n当前读数对比:\r\n");
            printf("             原始值    校准后   |M|\r\n");
            printf("  X轴:    %6d    %6d\r\n", raw.X, calibrated.X);
            printf("  Y轴:    %6d    %6d\r\n", raw.Y, calibrated.Y);
            printf("  Z轴:    %6d    %6d\r\n", raw.Z, calibrated.Z);
            printf("  强度:   %.4f    %.4f Gauss\r\n", raw_mag, cal_mag);
            
            if(cal_mag < 0.3f || cal_mag > 0.8f) {
                printf("  [!] 校准后磁场强度异常！建议重新校准\r\n");
            } else {
                printf("  [OK] 校准后磁场强度正常\r\n");
            }
        }
    }
    
    printf("\r\n");
}


