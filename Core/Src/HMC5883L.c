#include"HMC5883L.h"
#include "math.h"


uint8_t HMC_init(HMC_t *hmc,I2C_HandleTypeDef *i2c,uint8_t Output_Data_Rate)
{
	uint8_t array[2];
	hmc->i2c=i2c;
	hmc->Control_Register=0x11;
	array[0]=1;
	array[1]=hmc->Control_Register;

	if(Output_Data_Rate==200)hmc->Control_Register|=0b00001100;
	else if(Output_Data_Rate==100)hmc->Control_Register|=0b00001000;
	else if(Output_Data_Rate==50)hmc->Control_Register|=0b00000100;
	else if(Output_Data_Rate==10)hmc->Control_Register|=0b00000000;
	else hmc->Control_Register|=0b00001100;

	if(HAL_I2C_Mem_Write(hmc->i2c, 0x1A, 0x0B, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	if(HAL_I2C_Mem_Write(hmc->i2c, 0x1A, 0x09, 1, &array[1], 1, 100)!=HAL_OK)return 1;

	return 0;
}

uint8_t HMC_read(HMC_t *hmc)
{
	  hmc->datas[0]=0;
	  HAL_I2C_Mem_Read(hmc->i2c, 0x1A, 0x06, 1, hmc->datas, 1, 100);

	  if((hmc->datas[0]&0x01)==1)
	  {
		  HAL_I2C_Mem_Read(hmc->i2c, 0x1A, 0x00, 1, hmc->datas, 6, 100);
		  hmc->Xaxis= (hmc->datas[1]<<8) | hmc->datas[0];
		  hmc->Yaxis= (hmc->datas[3]<<8) | hmc->datas[2];
		  hmc->Zaxis= (hmc->datas[5]<<8) | hmc->datas[4];

		  hmc->compas=atan2f(hmc->Yaxis,hmc->Xaxis)*180.00/M_PI;

		  if(hmc->compas>0)
		  {
			  hmc->heading= hmc->compas;
		  }
		  else
		  {
			  hmc->heading=360+hmc->compas;
		  }
	  }
	  else
	  {
		  return 1;
	  }
return 0;
}

float HMC_readHeading(HMC_t *hmc)
{
	HMC_read(hmc);
	return hmc->heading;
}

uint8_t hmc_Reset(HMC_t *hmc)
{
	uint8_t array[1]={0x80};
	if(HAL_I2C_Mem_Write(hmc->i2c, 0x1A, 0x0A, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	return 0;
}
