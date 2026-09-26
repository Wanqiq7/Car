#pragma once

/**
 * @file mpu_port.h
 * @brief DMP 库与本工程的适配层（C 接口部分）/ Port layer for the InvenSense DMP
 *        library (C interface).
 *
 * 原 DMP 资料包（inv_mpu / inv_mpu_dmp_motion_driver / mpu6050）依赖 STM32 标准外设库：
 *   - 整型别名 u8/u16/u32… 来自 sys.h / stm32f10x.h
 *   - delay_ms()            来自 Delay.h
 *   - I2C                   由 MPU6050_I2C.c 用 GPIO 位操作软件模拟（PB10/PB11）
 *
 * 本工程基于 STM32 HAL + LibXR，且硬件 I2C1 已配置在 PB6(SCL)/PB7(SDA)。因此这里补齐
 * 类型与延时声明，并由 mpu_port.cpp 把 I2C 读写转发到 LibXR::I2C（硬件 I2C1）。
 *
 * The upstream DMP package is SPL-based and bit-bangs I2C on PB10/PB11. This project is
 * HAL + LibXR based with hardware I2C1 wired to PB6(SCL)/PB7(SDA), so this header supplies
 * the missing types/delay declarations and mpu_port.cpp routes I2C to LibXR::I2C.
 */

#include <stdint.h>

/* ---- 原 sys.h / stm32f10x.h 提供的整型别名 / Integer aliases from the original SPL ---- */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 毫秒级阻塞延时（对应原 Delay.h 的 delay_ms）
 *        Millisecond blocking delay (formerly `delay_ms` from Delay.h).
 * @param ms 延时时长（毫秒）/ Delay length in milliseconds.
 */
void delay_ms(u32 ms);

/**
 * @brief 器件级 I2C 总线准备（对应原 MPU6050_IIC_IO_Init）
 *        Device-side I2C bus preparation (formerly `MPU6050_IIC_IO_Init`).
 *
 * @note 使用硬件 I2C1 时总线已由 CubeMX 的 `MX_I2C1_Init()` 初始化，这里仅作为兼容占位，
 *       不做任何引脚操作。inv_mpu.c 的初始化流程仍会调用它。
 *       The bus is already brought up by `MX_I2C1_Init()`, so this is a no-op kept for
 *       compatibility with the upstream init sequence.
 */
void MPU6050_IIC_IO_Init(void);

/**
 * @brief 向寄存器写入若干字节（对应原 mpu6050_write）
 *        Write bytes to device registers (formerly `mpu6050_write`).
 * @param addr 7 位从机地址（不含 R/W 位），MPU6050 典型值 0x68
 *             7-bit slave address without the R/W bit; 0x68 for MPU6050.
 * @param reg  起始寄存器地址 / First register address.
 * @param len  写入字节数 / Number of bytes to write.
 * @param buf  数据缓冲区 / Source buffer.
 * @return 0 表示成功，非 0 表示失败（与原库语义一致）
 *         0 on success, non-zero on failure (same contract as upstream).
 */
int mpu_port_write(u8 addr, u8 reg, u8 len, const u8* buf);

/**
 * @brief 从寄存器读取若干字节（对应原 mpu6050_read）
 *        Read bytes from device registers (formerly `mpu6050_read`).
 * @param addr 7 位从机地址（不含 R/W 位），MPU6050 典型值 0x68
 *             7-bit slave address without the R/W bit; 0x68 for MPU6050.
 * @param reg  起始寄存器地址 / First register address.
 * @param len  读取字节数 / Number of bytes to read.
 * @param buf  接收缓冲区 / Destination buffer.
 * @return 0 表示成功，非 0 表示失败（与原库语义一致）
 *         0 on success, non-zero on failure (same contract as upstream).
 */
int mpu_port_read(u8 addr, u8 reg, u8 len, u8* buf);

#ifdef __cplusplus
}
#endif
