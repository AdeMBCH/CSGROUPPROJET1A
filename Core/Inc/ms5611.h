#include <stdint.h>
#include "stm32f4xx.h"

#define MS5611_I2C_ADDR			0x77	//Si pin csb est low (par défaut)
#define MS5611_I2C_ADDR2		0x76	//Si pin csb est high

#define MS5611_CMD_ADC_READ		0x00
#define MS5611_CMD_CONVERT_D1	0x40	//conversion pression
#define MS5611_CMD_CONVERT_D2	0x50	//conversion température
#define MS5611_CMD_RESET		0x1E
#define MS5611_CMD_READ_PROM	0xA2	//les valeurs de calibrations vont de 0xA2 à 0xAC avec un intervalle de 2

enum MS5611_OSR {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512,
	MS5611_OSR_1024,
	MS5611_OSR_2048,
	MS5611_OSR_4096,
};

static I2C_HandleTypeDef* ms5611_i2cx;

void ms5611_set_i2c(I2C_HandleTypeDef* i2cx);

uint8_t ms5611_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output);
uint8_t ms5611_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input);

void ms5611_osr_select(enum MS5611_OSR osr);

void ms5611_init();

void ms5611_update_pressure();
void ms5611_update_temperature();

void ms5611_update();

double ms5611_get_temperature();
double ms5611_get_pressure();
