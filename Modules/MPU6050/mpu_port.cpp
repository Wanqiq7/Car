/**
 * @file mpu_port.cpp
 * @brief DMP 库与本工程的适配层实现 / Implementation of the DMP port layer.
 *
 * 把 DMP 库的寄存器读写转发到 LibXR::I2C（硬件 I2C1），并把库内的 delay_ms 映射到
 * LibXR 的毫秒阻塞延时。
 * Routes the DMP library's register accesses to LibXR::I2C (hardware I2C1) and maps the
 * library's `delay_ms` onto LibXR's millisecond blocking sleep.
 */

#include "mpu_port.hpp"

#include "libxr.hpp"

namespace
{
/**
 * @brief 当前绑定的 I2C 实例 / Currently bound I2C instance.
 *
 * @note DMP 库自身（inv_mpu.c）就是全局单例状态（`static struct mpu_state_s st`），
 *       本工程也只挂载一颗 MPU6050，因此这里保持同样的单实例约定。
 *       The DMP library itself keeps global singleton state, and this project mounts a
 *       single MPU6050, so a single binding matches upstream semantics.
 */
LibXR::I2C* g_i2c = nullptr;

/* BLOCK 模式读写所需的一次性协作原语（同一时刻只服务一次调用）。
   One-shot coordination primitives for BLOCK-mode transfers (one call at a time). */
LibXR::Semaphore g_i2c_sem;
LibXR::ReadOperation g_i2c_read_op(g_i2c_sem);
LibXR::WriteOperation g_i2c_write_op(g_i2c_sem);
}  // namespace

void mpu_port_bind(LibXR::I2C* i2c) { g_i2c = i2c; }

extern "C" int mpu_port_write(u8 addr, u8 reg, u8 len, const u8* buf)
{
  if (g_i2c == nullptr)
  {
    return 1;
  }

  const auto EC =
      g_i2c->MemWrite(addr, reg, LibXR::ConstRawData(buf, len), g_i2c_write_op);
  return (EC == LibXR::ErrorCode::OK) ? 0 : 1;
}

extern "C" int mpu_port_read(u8 addr, u8 reg, u8 len, u8* buf)
{
  if (g_i2c == nullptr)
  {
    return 1;
  }

  const auto EC = g_i2c->MemRead(addr, reg, LibXR::RawData(buf, len), g_i2c_read_op);
  return (EC == LibXR::ErrorCode::OK) ? 0 : 1;
}

extern "C" void delay_ms(u32 ms) { LibXR::Thread::Sleep(ms); }

extern "C" void MPU6050_IIC_IO_Init(void)
{
  /* 硬件 I2C1 已在 MX_I2C1_Init() 中完成初始化，此处无需任何操作。 */
  /* Hardware I2C1 is already initialised by MX_I2C1_Init(); nothing to do here. */
}
