#include "MPU6050.h"


static I2C_HandleTypeDef i2cHandler;

static float accelScalingFactor, gyroScalingFactor;


static float A_X_Bias = 0.0f;
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;

static int16_t GyroRW[3];


void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Reprend l'I2C de CubeMX handle pour le ramener ici
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

void I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	//Décalage d'un bit vers la gauche pour la réception I2C
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 100);
}

void I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,100);
}

void MPU6050_Config(MPU_ConfigTypeDef *config)
{

	uint8_t Buffer = 0;
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //changement du 7eme bit du registre
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // changement unique du 7 bit
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // attente de 100ms avant de changer le réglage d'horloge askip c'est nécessaire :)
	
	//Mise en place du filtre passe bas
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);
	
	//Full scale range du Gyro
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);
	
	//Full scale range de l'accéléro
	Buffer = 0; 
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//SRD en Default
	MPU6050_Set_SMPRT_DIV(0x04);
	

	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
			break;
		
		default:
			break;
	}


	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;
		
		default:
			break;
	}
	
}


uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}


void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}


uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}


void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(CONFIG_REG, Buffer);
	
}


void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];
	
	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1]&&0x01))
	{
		I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);
		
		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);
	
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
}

void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(&AccelScaled);
	
	CaliDef->x = (AccelScaled.x) - A_X_Bias;
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;
}

void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{
	
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];
	
}

void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);
	
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor;
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor;
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor;
}


void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	A_X_Bias= (x_max + x_min)/2.0f;
	A_Y_Bias= (y_max + y_min)/2.0f;
	A_Z_Bias= (z_max + z_min)/2.0f;
}
