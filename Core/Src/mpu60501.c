/* Version adapatée du filtre de Kalman utilisé dans https://github.com/TKJElectronics/KalmanFilter*/


#include <math.h>
#include "mpu60501.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU60501_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 est return si tout va bien
    {
        // registre du power managment 0X6B on réveil le capteur en mettant des O
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // DATA RATE à 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Configuration de l'accéléromètre
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Configuration du gyroscope
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *MPUdata)
{
    uint8_t Rec_Data[6];

    // Lecture

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    MPUdata->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPUdata->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPUdata->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    //conversion des données brutes en acceleration en g donc on divise par rapport aux valeurs mises dans FS_SEL, soit division par 16384.0 parce que FS_SEL=0

    MPUdata->Ax = MPUdata->Accel_X_RAW / 16384.0;
    MPUdata->Ay = MPUdata->Accel_Y_RAW / 16384.0;
    MPUdata->Az = MPUdata->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *MPUdata)
{
    uint8_t Rec_Data[6];

    // Lecture

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    MPUdata->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPUdata->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPUdata->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // conversion en dps donc on divise par 131.0 pour correspondre à FS_SEL

    MPUdata->Gx = MPUdata->Gyro_X_RAW / 131.0;
    MPUdata->Gy = MPUdata->Gyro_Y_RAW / 131.0;
    MPUdata->Gz = MPUdata->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *MPUdata)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Lecture pour la température de la mpu

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPUdata->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *MPUdata)
{
    uint8_t Rec_Data[14];
    int16_t temp;


    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    MPUdata->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPUdata->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPUdata->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    MPUdata->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    MPUdata->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    MPUdata->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    MPUdata->Ax = MPUdata->Accel_X_RAW / 16384.0;
    MPUdata->Ay = MPUdata->Accel_Y_RAW / 16384.0;
    MPUdata->Az = MPUdata->Accel_Z_RAW / Accel_Z_corrector;
    MPUdata->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    MPUdata->Gx = MPUdata->Gyro_X_RAW / 131.0;
    MPUdata->Gy = MPUdata->Gyro_Y_RAW / 131.0;
    MPUdata->Gz = MPUdata->Gyro_Z_RAW / 131.0;

    // Kalman pour les angles
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        MPUdata->Accel_X_RAW * MPUdata->Accel_X_RAW + MPUdata->Accel_Z_RAW * MPUdata->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(MPUdata->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-MPUdata->Accel_X_RAW, MPUdata->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && MPUdata->KalmanAngleY > 90) || (pitch > 90 && MPUdata->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        MPUdata->KalmanAngleY = pitch;
    }
    else
    {
        MPUdata->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, MPUdata->Gy, dt);
    }
    if (fabs(MPUdata->KalmanAngleY) > 90)
        MPUdata->Gx = -MPUdata->Gx;
    MPUdata->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, MPUdata->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
