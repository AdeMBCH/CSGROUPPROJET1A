#include "ms5611.h"
#include "i2c.h"
#include "math.h"

#define NUM_CALIBRATION_DATA 6

uint16_t fc[NUM_CALIBRATION_DATA];
uint32_t raw_pressure, raw_temperature;
enum MS5611_OSR selected_osr = MS5611_OSR_4096;

void ms5611_set_i2c(I2C_HandleTypeDef* i2cx){
	ms5611_i2cx = i2cx;
}

uint8_t ms5611_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output){
	return I2C_read(ms5611_i2cx,MS5611_I2C_ADDR,register_address,length,output);
}

uint8_t ms5611_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input){
	return I2C_write(ms5611_i2cx,MS5611_I2C_ADDR,register_address,length,input);
}

/**
 * ADC resolution, plusieurs résolutions voir datasheet allant de MS5611_OSR_256 à MS5611_OSR_4096
 */
void ms5611_osr_select(enum MS5611_OSR osr){
	selected_osr = osr;
}

void ms5611_init(){
	//read 6 factory calibration data
	for (int i = 0; i < NUM_CALIBRATION_DATA; i++){
		uint8_t reg_addr = MS5611_CMD_READ_PROM + (i << 1);//interval 2
		uint8_t buffer[2];
		ms5611_read_i2c(reg_addr,2,buffer);

		fc[i] = (uint16_t)(buffer[0] << 8 | buffer[1]);
	}
}

void ms5611_update_pressure(){
	uint8_t buffer[3] = {0x00,0x00,0x00};
	int state;
	state = ms5611_write_i2c(MS5611_CMD_CONVERT_D1 | (selected_osr << 1),0,buffer);

	HAL_Delay(12);//Temps nécessaire pour la conversion ADCn doit être >= 9.02ms

	state = ms5611_read_i2c(MS5611_CMD_ADC_READ,3,buffer);
	raw_pressure = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);

}

void ms5611_update_temperature(){
	uint8_t buffer[3] = {0x00,0x00,0x00};
	int state;
	state = ms5611_write_i2c(MS5611_CMD_CONVERT_D2 | (selected_osr << 1),0,buffer);

	HAL_Delay(12);//Temps nécessaire pour la conversion ADCn doit être >= 9.02ms

	state = ms5611_read_i2c(MS5611_CMD_ADC_READ,3,buffer);
	raw_temperature = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
}

void ms5611_update(){
	ms5611_update_temperature();
	ms5611_update_pressure();
}

double ms5611_get_temperature(){
	uint32_t dT = raw_temperature - ((uint32_t)fc[4] * 256);
	double TEMP = 2000.0 + dT * (fc[5] / (8388608.0));//unité 0.01 C

	double T2=0;
	if (TEMP < 2000){
		//temperature < 20 degré Celsius
		T2 = dT * (dT / (2147483648.0));
	}

	TEMP = TEMP - T2;
	return TEMP / 100;
}

double ms5611_get_pressure(){
	uint32_t dT = raw_temperature - ((uint32_t)fc[4] * 256);
	double TEMP = 2000.0 + dT * (fc[5] / (8388608.0));//unité 0.01 C

	double OFF = fc[1] * (65536) + fc[3] * dT / (128);
	double SENS = fc[0] * (32768) + fc[2] * dT / (256);

	double P = (raw_pressure * SENS / (2097152.0) - OFF) / (32768);//unité 0.01mbar

	double T2=0, OFF2=0, SENS2=0;
	if (TEMP < 2000){
		T2 = dT * dT / (2147483648);
		OFF2 = 5 * (TEMP-2000) * (TEMP-2000) / 2;
		SENS2 = 5 * (TEMP-2000) * (TEMP-2000) / 4;

		if (TEMP < -1500){
			//temperature < -15 degré Celsius
			OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 = SENS2 + 11/2 * (TEMP + 1500) * (TEMP + 1500);
		}
	}

	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	P = (raw_pressure * SENS / (2097152.0) - OFF) / (32768);
	return P / 100;
}
