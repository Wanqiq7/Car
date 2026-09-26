# MPU6050

InvenSense MPU6050 六轴 IMU 驱动模块，包含片内 DMP（Digital Motion Processor）姿态解算。
模块基于 DMP 资料包中的 InvenSense 驱动库移植，I2C 后端改为 LibXR 抽象。

InvenSense MPU6050 6-axis IMU driver module with on-chip DMP attitude estimation, ported
from the upstream InvenSense DMP package with the I2C backend rebased onto LibXR.

## 硬件依赖 / Hardware requirements

| 资源 | 说明 |
|------|------|
| `i2c_mpu6050` / `i2c1` / `I2C1` | 硬件 I2C1，`PB6 = SCL`，`PB7 = SDA`（400 kHz 快速模式） |
| `ramfs` | 调试命令挂载点 / Mount point for the debug command |

从机地址为 7 位 `0x68`（不含 R/W 位）。ADC、UART、DMA 等其余外设与本模块无关。

Slave address is the 7-bit value `0x68` (no R/W bit). Other peripherals are unrelated.

## 构造参数 / Constructor arguments

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gyro_topic_name` | `"mpu6050_gyro"` | 角速度 Topic，单位 `rad/s` |
| `accl_topic_name` | `"mpu6050_accl"` | 加速度 Topic，单位 `m/s^2` |
| `euler_topic_name` | `"mpu6050_euler"` | 欧拉角 Topic，单位 `deg`（顺序 pitch / roll / yaw） |
| `sample_period_ms` | `10` | FIFO 采集周期（毫秒），默认对齐 DMP 的 100 Hz 输出率 |

## 输出 / Published topics

三个 Topic 的负载均为 `Eigen::Matrix<float, 3, 1>`，并带 `LibXR::MicrosecondTimestamp`：

- `gyro`：角速度，`rad/s`。DMP 的 `DMP_FEATURE_SEND_CAL_GYRO` 已做零偏补偿。
- `accl`：重力加速度，`m/s^2`。
- `euler`：由 DMP 四元数换算的姿态角，`deg`。

标度换算基于 `inv_mpu.c` 中 `mpu_init()` 的固定配置：陀螺仪 ±2000 dps（16.4 LSB/dps）、
加速度计 ±2 g（16384 LSB/g）。

All three payloads are `Eigen::Matrix<float, 3, 1>` published with a
`LibXR::MicrosecondTimestamp`. Scaling follows the fixed `mpu_init()` configuration in
`inv_mpu.c`: gyro ±2000 dps (16.4 LSB/dps), accel ±2 g (16384 LSB/g).

## 调试命令 / Debug command

```
mpu6050 whoami
mpu6050 show [duration_ms] [interval_ms]
```

`show` 输出的三组量一律为**定标整数**（不经过浮点格式化）：

| 列 | 单位 | 定标 | 示例含义 |
|---|---|---|---|
| `Euler(deg*100)` | 0.01 deg | ×100 | `+1234` = 12.34° |
| `Gyro(rad/s*1e4)` | 0.0001 rad/s | ×10000 | `+123` = 0.0123 rad/s |
| `Accl(m/s^2*1e3)` | 0.001 m/s² | ×1000 | `+9812` = 9.812 m/s² |

四舍五入采用"远离零"策略（`ScaleToInt()`）。改用整数输出是为了让调试通道不依赖浮点格式化路径、
结果确定可预期；代价约 +204 B Flash（`.text` +132、`.rodata` +72），相对 64 KB 预算可接受。

The three groups printed by `show` are **scaled integers** rather than floats (units above).
Rounding is round-half-away-from-zero. This keeps the debug path independent of float
formatting; it costs about +204 bytes of flash.

## Ozone 观察 / Ozone watch

模块提供一段**扁平调试镜像**，避免在 Ozone 的 Watched Data 中书写 Eigen 的三层嵌套路径
（`euler_.m_storage.m_data`）：

```cpp
inline static struct {
  float gyro[3];
  float accl[3];
  float euler[3];
  uint32_t sample_count;
} MPU6050_data{};
```

Watched Data 中直接填：

```
MPU6050::MPU6050_data
```

展开即 3 组 × 3 个 `float` 加 1 个计数器；浮点部分与发布到 Topic 的数据完全一致
（每次成功采样同步一次）：

| 成员 | 偏移 | 单位 / 说明 |
|---|---|---|
| `gyro[3]` | 0 | `rad/s` |
| `accl[3]` | 12 | `m/s^2` |
| `euler[3]` | 24 | `deg`（pitch / roll / yaw） |
| `sample_count` | 36 | **诊断计数**，无单位（`uint32_t`） |

镜像位于 `.bss`，符号形如 `_ZN7MPU605012MPU6050_dataE`（40 字节）。

### `sample_count` 的语义 / Semantics of `sample_count`

`sample_count` 在 `SampleTaskFun` **入口处**自增，因此与 `Poll()` 成败无关：它计的是
“周期任务被调度了多少次”，而不是“成功取到了多少帧”。这一点是关键，因为它能把
“数据全 0”这个单一现象拆成两种互斥的情形：

| 观察到的现象 | 推断 |
|---|---|
| `sample_count` **停止不动** | 定时任务从未被调度：构造期卡在 `MPU6050.hpp` 的 `while (!Init())`，或目标未运行当前镜像 |
| `sample_count` **递增但 gyro/accl/euler 仍为 0** | 任务在跑，但 `dmp_read_fifo()` 每次都取不到带四元数的新包 |
| `sample_count` 递增且浮点非 0 | 采集链路正常 |

计数为 `uint32_t` 回绕，在 `sample_period_ms = 1` 时约以 kHz 速率递增（实际受 I2C 事务
耗时限制），约 49.7 天回绕一次，不会影响诊断。

**需要带调试信息的预设**：`RelWithDebInfo`（`-Os -g3`，体积与 `Release` 相同）才会生成该
变量的 DWARF 类型信息；`Release`（`-Os -g0`）下符号仍在符号表中，但没有类型信息，Ozone
无法展开命名成员。`RelWithDebInfo` 自 2026-09-26 起已是 `build.ps1` 的默认预设，日常构建即满足此要求。

A flat debug mirror is published so that Ozone's Watched Data needs only
`MPU6050::MPU6050_data` instead of Eigen's nested storage path. The mirror lives in `.bss`
and is refreshed once per successful sample. A preset carrying debug information
(`RelWithDebInfo`) is required for Ozone to resolve the named members.

## 文件构成 / File layout

| 文件 | 来源 / 说明 |
|------|-------------|
| `inv_mpu.c/.h`、`inv_mpu_dmp_motion_driver.c/.h`、`dmpKey.h`、`dmpmap.h` | 上游 DMP 库，逐字节保留；仅 `inv_mpu.h` / `inv_mpu.c` 各一行 `#include` 改为 `mpu_port.h` |
| `MPU6050.c/.h` | 上游寄存器层，逐函数等价移植，I2C 原语改走 `mpu_port` |
| `mpu_port.h/.hpp/.cpp` | 适配层：整型别名、`delay_ms`、I2C 转发到 `LibXR::I2C` |
| `MPU6050.hpp` | XRobot 模块（`LibXR::Application`），Topic 发布与调试命令 |

The upstream DMP sources are kept byte-for-byte identical; only one `#include` line each in
`inv_mpu.h` and `inv_mpu.c` is redirected to `mpu_port.h`.

## 注意事项 / Notes

- 上游源码中 `inv_mpu.c`、`inv_mpu.h`、`MPU6050.c`、`MPU6050.h` 的中文注释为 **GBK** 编码，
  本模块刻意保留原始字节以维持与上游的可对比性；GCC 默认输入编码可正确接受这些注释。
- 采样由 `LibXR::Timer` 驱动。本工程 `SYSTEM` 后端为 `None`（裸机协作式），
  `LibXR::Thread::Create()` 会同步执行目标函数，因此模块不使用常驻线程。
- I2C 传输走 `LibXR::I2C` 的 BLOCK（阻塞）路径。本模块最大单次传输是 DMP FIFO 整包
  **32 字节**（6 加速度 + 6 陀螺仪 + 16 四元数 + 4 手势），与 `i2c1_buf[32]` 等长；
  `STM32I2C` 的 `dma_enable_min_size` 取 32，使 `size > min` 恒为假，永远不会进入
  未配置 DMA 的分支。若改动 DMP 特征掩码使包长超过 32 字节，必须同步增大缓冲区与阈值。
- `MPU6050_DMP_Init()` 对失败同样返回 0，模块内部以实际读取一帧 DMP 数据来判定初始化结果。

The four files with Chinese comments are GBK-encoded upstream and are kept byte-identical
on purpose; GCC accepts them under the default input charset. Sampling is timer-driven
because this project uses the bare-metal `None` system backend, where
`LibXR::Thread::Create()` runs its target synchronously. The I2C path stays on LibXR's
BLOCK transfers: the largest single transfer is the 32-byte DMP FIFO packet, which equals
`i2c1_buf[32]`, so `dma_enable_min_size` is set to 32 and the unconfigured DMA branch is
never taken.
