#pragma once

/**
 * @file mpu_port.hpp
 * @brief DMP 库与本工程的适配层（C++ 接口部分）/ Port layer for the InvenSense DMP
 *        library (C++ interface).
 */

#include "i2c.hpp"
#include "mpu_port.h"

/**
 * @brief 绑定 MPU6050 使用的 LibXR I2C 实例
 *        Bind the LibXR I2C instance used by the MPU6050.
 *
 * @param i2c 由 HardwareContainer 提供的 I2C 对象（本工程为硬件 I2C1，PB6=SCL/PB7=SDA）
 *            I2C object resolved from the HardwareContainer (hardware I2C1 here).
 *
 * @note 必须在调用 `MPU6050_DMP_Init()` 之前调用一次，否则所有寄存器访问都会失败。
 *       Must be called once before `MPU6050_DMP_Init()`, otherwise every register access
 *       fails.
 */
void mpu_port_bind(LibXR::I2C* i2c);
