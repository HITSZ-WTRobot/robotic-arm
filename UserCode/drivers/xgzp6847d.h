#ifndef XGZP6847D_H
#define XGZP6847D_H

#include "cmsis_os2.h"
#include "i2c.h"


#define XGZP6847D_SLAVE_ADDRESS 0x6D // I2C address of the slave
#define PRESSURE_REG            0x06 // Pressure register address
#define TEMPERATURE_REG         0x09 // Temperature register address

#ifdef __cplusplus

class XGZP6847D
{
public:
    static const uint8_t SENSOR_ADDRESS =
        XGZP6847D_SLAVE_ADDRESS;                              // XGZP6847D I2C地址
    XGZP6847D(I2C_HandleTypeDef* hi2c, float Pressure_Range); // XGZP6847D构造函数
    float readPressure();                                     // 读取气压：Pa
    float readTemperature();                                  // 读取温度：摄氏度
private:
    I2C_HandleTypeDef* _hi2c; // I2C句柄
    float _K;                 // 根据量程计算的压力转换系数
};
#endif // __cplusplus
#endif // XGZP6847D_H
