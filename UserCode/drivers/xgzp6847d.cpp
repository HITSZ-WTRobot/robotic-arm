/**
 * @file    xgzp6847d.cpp
 * @author  ChillyHigh
 * @date    2025-11-14
 * @brief   Brief description of the file. 不使用TCA9548A
 */

#include "drivers/xgzp6847d.h"

//
//    FILE: XGZP6847D.cpp
//  AUTHOR: Cédric Rey
// VERSION: 1.1
//    DATE: 2024-02-20
// PURPOSE: ESP / Arduino library for XGZP Digital Pressure sensor
//     URL: https://github.com/tuxmountain/XGZP_Pressure_Sensor_Digital
//

// XGZP6847D 类构造函数
XGZP6847D::XGZP6847D(I2C_HandleTypeDef* hi2c, float Pressure_Range)
{
    _hi2c = hi2c;
    // 根据压力量程 (kPa) 选择 _K 值
    if (Pressure_Range > 500 && Pressure_Range <= 1000)
    {
        _K = 8;
    }
    else if (Pressure_Range > 260)
    {
        _K = 16;
    }
    else if (Pressure_Range > 130)
    {
        _K = 32;
    }
    else if (Pressure_Range > 65)
    {
        _K = 64;
    }
    else if (Pressure_Range > 32)
    {
        _K = 128;
    }
    else if (Pressure_Range > 16)
    {
        _K = 256;
    }
    else if (Pressure_Range > 8)
    {
        _K = 512;
    }
    else if (Pressure_Range > 4)
    {
        _K = 1024;
    }
    else if (Pressure_Range > 2)
    {
        _K = 2048;
    }
    else
    {
        _K = 4096; // 适用于 1 ≤ P ≤ 2
    }
}

// 读取气压：Pa
float XGZP6847D::readPressure()
{
    int pressure;
    uint8_t cmd = 0x0A;
    // 发送 0x30 指示进行组合转换, 发送 0x0A
    HAL_I2C_Mem_Write(_hi2c, SENSOR_ADDRESS << 1, 0x30, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);

    // 等待转换完成
    uint8_t status = 0;
    while (true)
    {
        // 读取状态寄存器 0x30
        HAL_I2C_Mem_Read(_hi2c, SENSOR_ADDRESS << 1, 0x30, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
        if ((status & 0x08) == 0)
        {
            break;
        }
    }

    uint8_t data[3];
    // 读取压力数据（3 字节）
    if (HAL_I2C_Mem_Read(_hi2c, SENSOR_ADDRESS << 1, PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, data, 3, 100) == HAL_OK)
    {
        uint8_t pressure_H = data[0]; // 读取高字节
        uint8_t pressure_M = data[1]; // 读取中字节
        uint8_t pressure_L = data[2]; // 读取低字节

        // 将三个字节转换成原始压力值
        long int rawData = pressure_H * 65536 + pressure_M * 256 + pressure_L;

        if (rawData & 0x800000)
        {                                           // 压力值为负
            pressure = (rawData - (1L << 24)) / _K; // 负压换算公式
        }
        else
        {
            pressure = rawData / _K; // 正压换算公式
        }

        return (float)pressure; // 返回压力（单位：Pa）
    }
    else
    {
        // 处理读取错误或返回特定错误值
        return -999.0; // 错误值
    }
}

// 读取温度：摄氏度
float XGZP6847D::readTemperature()
{
    uint8_t data[2];
    // 读取温度数据（2 字节）
    if (HAL_I2C_Mem_Read(_hi2c, SENSOR_ADDRESS << 1, TEMPERATURE_REG, I2C_MEMADD_SIZE_8BIT, data, 2, 100) == HAL_OK)
    {
        uint8_t temperature_H = data[0]; // 读取高字节
        uint8_t temperature_L = data[1]; // 读取低字节

        // 将两个字节转换成原始温度值
        long int rawData = ((long)temperature_H << 8) | temperature_L;

        // 将原始数据换算为摄氏温度
        float temperature;
        if (rawData > 32767)
        {
            temperature = (rawData - 65536) / 256.0;
        }
        else
        {
            temperature = rawData / 256.0;
        }
        return temperature; // 返回温度（单位：℃）
    }
    else
    {
        // 处理读取错误或返回特定错误值
        return -999.0; // 错误示例值
    }
}
