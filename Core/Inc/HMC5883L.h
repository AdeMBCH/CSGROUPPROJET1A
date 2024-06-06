#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "main.h"
#include"stm32f4xx.h"




#define Standby 0
#define Continuous 1
#define HMC_OK 0
#define HMC_FALSE 1





typedef struct HMC
{
	I2C_HandleTypeDef   *i2c;
	uint8_t				Control_Register;
	uint8_t             datas[6];
	int16_t             Xaxis;
	int16_t             Yaxis;
	int16_t             Zaxis;
	float			    heading;
	float               compas;
}HMC_t;







uint8_t HMC_init(HMC_t *hmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate);
uint8_t HMC_read(HMC_t *hmc);
float   HMC_readHeading(HMC_t *hmc);
uint8_t HMC_Standby(HMC_t *hmc);
uint8_t HMC_Reset(HMC_t *hmc);

#endif
