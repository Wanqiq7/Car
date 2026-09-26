#ifndef _MPU6050_H_
#define _MPU6050_H_

/**
 * @file MPU6050.h
 * @brief MPU6050 寄存器映射与基础读写接口 / MPU6050 register map and raw access API.
 *
 * 移植自 DMP 资料包中的 mpu6050.h / mpu6050.c：寄存器定义保持完全一致，底层 I2C 由
 * GPIO 位操作软件模拟改为经 mpu_port 转发到 LibXR 硬件 I2C1。
 * Ported from the DMP package (mpu6050.h/mpu6050.c). Register definitions are unchanged;
 * the bit-banged GPIO I2C backend is replaced by mpu_port on top of LibXR hardware I2C1.
 */

#include "mpu_port.h"

#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F

#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器

#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	MPU_DEVICE_ID_REG	  	0x75
#define	MPU_ADDR	0x68 //IIC地址寄存器0x68

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- 与原始 mpu6050.c 等价的 I2C 基本读写 / Raw I2C access ---- */
u8   mpu6050_write(u8 addr, u8 reg, u8 len, const u8* buf); //返回值 0：写成功  非0：写失败
u8   mpu6050_read (u8 addr, u8 reg, u8 len, u8 *buf);       //返回值 0：读成功  非0：读失败
void mpu6050_write_reg(u8 reg, u8 dat);
u8   mpu6050_read_reg (u8 reg);

/* ---- 非 DMP 路径的原始配置与读取 API（保留原始接口，供调试/后备使用）----
   ---- Raw configuration/getter API kept from the original library ---- */
u8   MPU_Set_Gyro_Fsr(u8 fsr);
u8   MPU_Set_Accel_Fsr(u8 fsr);
u8   MPU_Set_LPF(u16 lpf);
u8   MPU_Set_Rate(u16 rate);
void MPU6050_Init(void);
short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#ifdef __cplusplus
}
#endif

#endif
