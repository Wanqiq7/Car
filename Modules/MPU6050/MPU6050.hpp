#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: InvenSense MPU6050 六轴 IMU 及片内 DMP 姿态解算驱动（DMP 库移植版）/ MPU6050 6-axis IMU driver with on-chip DMP motion processing
constructor_args:
  - gyro_topic_name: "mpu6050_gyro"
  - accl_topic_name: "mpu6050_accl"
  - euler_topic_name: "mpu6050_euler"
  - sample_period_ms: 10
template_args: []
required_hardware: i2c_mpu6050/i2c1/I2C1 ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "app_framework.hpp"
#include "i2c.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "timer.hpp"
#include "transform.hpp"

#include "mpu_port.hpp"

#include "MPU6050.h"

extern "C" {
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

/**
 * @brief MPU6050 + 片内 DMP 驱动模块 / MPU6050 driver module with on-chip DMP.
 *
 * @details
 * 以 PLDX 模块规范封装 InvenSense DMP 资料包的移植版本：构造期完成 MPU6050 初始化与 DMP
 * 固件加载，之后由软件定时器按 `sample_period_ms` 周期读取 DMP FIFO，并把姿态、角速度、
 * 加速度分别发布到 LibXR Topic。
 *
 * 底层 I2C 走 LibXR 抽象（本工程 = 硬件 I2C1，PB6=SCL/PB7=SDA），因此不再依赖标准外设库
 * 的 GPIO 位操作软件 I2C。
 *
 * @note 本工程 `SYSTEM` 后端为 `None`（裸机协作式），`LibXR::Thread::Create()` 会同步执行
 *       目标函数，因此这里与 `BlinkLED` 一致，采用 `LibXR::Timer` 驱动的周期任务，而不是
 *       常驻采集线程。
 *
 * Wraps the ported InvenSense DMP package following the PLDX module convention: the
 * constructor initialises the MPU6050 and loads the DMP firmware, then a software timer
 * polls the DMP FIFO every `sample_period_ms` and publishes attitude, angular rate and
 * acceleration to LibXR topics. I2C goes through LibXR (hardware I2C1 here), so the old
 * bit-banged SPL GPIO backend is gone.
 *
 * @note This project uses the bare-metal `None` system backend where
 *       `LibXR::Thread::Create()` runs its target synchronously, so sampling is driven by
 *       a `LibXR::Timer` task (same pattern as `BlinkLED`) instead of a worker thread.
 */
class MPU6050 : public LibXR::Application {
 public:
  using Vec3 = Eigen::Matrix<float, 3, 1>;

  /** 重力加速度 / Standard gravity (m/s^2). */
  static constexpr float K_G = 9.80665F;
  static constexpr float K_DEG2RAD = static_cast<float>(LibXR::PI / 180.0);
  static constexpr float K_RAD2DEG = static_cast<float>(180.0 / LibXR::PI);

  /* inv_mpu.c 的 mpu_init() 固定把陀螺仪设为 ±2000dps、加速度计设为 ±2g，
     因此 DMP FIFO 中 gyro/accel 的原始值按下列标度换算。
     inv_mpu.c's mpu_init() pins the gyro to ±2000dps and the accel to ±2g, so the raw
     FIFO values are scaled as follows. */
  static constexpr float K_GYRO_LSB_PER_DPS = 16.4F;
  static constexpr float K_ACCL_LSB_PER_G = 16384.0F;
  /** DMP 四元数为 q30 定点格式 / DMP quaternions are q30 fixed-point. */
  static constexpr float K_Q30 = 1073741824.0F;

  /* 调试命令的整数定标倍数：一律以整数打印，不经过浮点格式化路径。
     Integer scale factors used by the debug command so that no float formatting is
     involved. euler: 0.01 deg, gyro: 0.0001 rad/s, accl: 0.001 m/s^2. */
  static constexpr int K_EULER_SCALE = 100;
  static constexpr int K_GYRO_SCALE = 10000;
  static constexpr int K_ACCL_SCALE = 1000;

  /* ── 调试镜像 / Debug mirror ────────────────────────────────────────────────
     把调试时要观察的 9 个浮点量摊平成一段连续内存，避免在 Ozone 的 Watched Data
     里书写 Eigen 的三层嵌套路径（`euler_.m_storage.m_data`）。每次成功采样后同步
     一次，内容与发布到 Topic 的数据完全一致。
     在 Ozone 中直接观察 `MPU6050::MPU6050_data`，展开即为 gyro/accl/euler 三组各
     3 个 float（单位：rad/s、m/s^2、deg）。

     `sample_count` 是诊断用的调用计数：它在 `SampleTaskFun` **入口处**自增，
     因此与 `Poll()` 成败无关。用途是分辨两种“数据全 0”的情形——
     计数停止 ⇒ 周期任务根本没有被调度（构造期卡在 `while (!Init())`，或整机未运行）；
     计数递增而 gyro/accl/euler 仍为 0 ⇒ 任务在跑，但 `dmp_read_fifo()` 每次都取不到
     带四元数的新包。

     Flattens the nine floats needed for debugging into one contiguous block so that
     Ozone's Watched Data can use a single flat expression instead of Eigen's nested
     storage path. Refreshed once per successful sample; contents are identical to the
     data published to the topics. Units: rad/s, m/s^2, deg.
     `sample_count` is a diagnostic counter incremented at the *entry* of
     `SampleTaskFun`, i.e. independent of whether `Poll()` succeeds. */
  inline static struct {
    float gyro[3];
    float accl[3];
    float euler[3];
    uint32_t sample_count;
  } MPU6050_data{};

  /**
   * @brief 构造 MPU6050 模块 / Construct the MPU6050 module.
   * @param hw 硬件容器 / Hardware container.
   * @param app 应用管理器 / Application manager.
   * @param gyro_topic_name 角速度 Topic 名称（rad/s）/ Angular-rate topic (rad/s).
   * @param accl_topic_name 加速度 Topic 名称（m/s^2）/ Acceleration topic (m/s^2).
   * @param euler_topic_name 欧拉角 Topic 名称（deg）/ Euler-angle topic (deg).
   * @param sample_period_ms 采集周期（毫秒）/ Sampling period in milliseconds.
   */
  MPU6050(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const char* gyro_topic_name, const char* accl_topic_name,
          const char* euler_topic_name, uint32_t sample_period_ms)
      : topic_gyro_(LibXR::Topic::CreateTopic<Vec3>(gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<Vec3>(accl_topic_name)),
        topic_euler_(LibXR::Topic::CreateTopic<Vec3>(euler_topic_name)),
        i2c_(hw.template FindOrExit<LibXR::I2C>({"i2c_mpu6050", "i2c1", "I2C1"})),
        cmd_file_(LibXR::RamFS::CreateFile("mpu6050", CommandFunc, this)) {
    app.Register(*this);

    /* DMP 库的所有寄存器访问都必须在此之后进行。 */
    mpu_port_bind(i2c_);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    while (!Init()) {
      XR_LOG_ERROR("MPU6050: DMP init failed, retrying...");
      LibXR::Thread::Sleep(500);
    }
    XR_LOG_PASS("MPU6050: DMP init succeeded.");

    timer_handle_ = LibXR::Timer::CreateTask(SampleTaskFun, this, sample_period_ms);
    LibXR::Timer::Add(timer_handle_);
    LibXR::Timer::Start(timer_handle_);
  }

  /**
   * @brief 初始化器件并加载 DMP 固件 / Initialise the device and load the DMP firmware.
   * @return 初始化并成功取到一帧 DMP 数据返回 true，否则 false
   *         true once a DMP frame has been read successfully, false otherwise.
   */
  bool Init() {
    if (mpu6050_read_reg(MPU_DEVICE_ID_REG) != MPU_ADDR) {
      return false;
    }

    /* 该函数内部完成 mpu_init / FIFO 配置 / DMP 固件加载 / 自检 / DMP 使能。
       注意它对失败也返回 0，所以下面用实际取数来判定是否真的可用。
       It also returns 0 on failure, hence the explicit frame-read check below. */
    MPU6050_DMP_Init();

    for (int i = 0; i < 20; ++i) {
      LibXR::Thread::Sleep(10);
      if (Poll() == 0) {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 从 DMP FIFO 取一帧并换算为物理量 / Read one DMP frame and convert units.
   * @return 0 成功；-1 无新数据；-2 四元数缺失
   *         0 on success, -1 when the FIFO has no new packet, -2 when the quaternion is
   *         absent.
   */
  int Poll() {
    short gyro_raw[3] = {0};
    short accl_raw[3] = {0};
    short sensors = 0;
    unsigned char more = 0;
    long quat[4] = {0};
    unsigned long timestamp = 0;

    if (dmp_read_fifo(gyro_raw, accl_raw, quat, &timestamp, &sensors, &more) != 0) {
      return -1;
    }

    if ((sensors & INV_XYZ_GYRO) != 0) {
      gyro_ = Vec3(static_cast<float>(gyro_raw[0]), static_cast<float>(gyro_raw[1]),
                   static_cast<float>(gyro_raw[2])) *
              (K_DEG2RAD / K_GYRO_LSB_PER_DPS);
    }

    if ((sensors & INV_XYZ_ACCEL) != 0) {
      accl_ = Vec3(static_cast<float>(accl_raw[0]), static_cast<float>(accl_raw[1]),
                   static_cast<float>(accl_raw[2])) *
              (K_G / K_ACCL_LSB_PER_G);
    }

    if ((sensors & INV_WXYZ_QUAT) == 0) {
      return -2;
    }

    const float Q0 = static_cast<float>(quat[0]) / K_Q30;
    const float Q1 = static_cast<float>(quat[1]) / K_Q30;
    const float Q2 = static_cast<float>(quat[2]) / K_Q30;
    const float Q3 = static_cast<float>(quat[3]) / K_Q30;

    /* pitch / roll / yaw，与 inv_mpu.c 中 MPU6050_DMP_Get_Data 使用同一组公式。
       Same formulas as MPU6050_DMP_Get_Data() in inv_mpu.c. */
    euler_ = Vec3(std::asin(-2.0F * Q1 * Q3 + 2.0F * Q0 * Q2) * K_RAD2DEG,
                  std::atan2(2.0F * Q2 * Q3 + 2.0F * Q0 * Q1,
                             -2.0F * Q1 * Q1 - 2.0F * Q2 * Q2 + 1.0F) *
                      K_RAD2DEG,
                  std::atan2(2.0F * (Q1 * Q2 + Q0 * Q3),
                             Q0 * Q0 + Q1 * Q1 - Q2 * Q2 - Q3 * Q3) *
                      K_RAD2DEG);
    return 0;
  }

  /**
   * @brief 周期采样任务 / Periodic sampling task.
   * @param self 模块实例 / Module instance.
   */
  static void SampleTaskFun(MPU6050* self) {
    /* 入口计数：与 Poll() 成败无关，用于判断本任务是否真的被调度。
       Entry counter: independent of Poll(), tells whether this task is scheduled. */
    ++MPU6050_data.sample_count;

    if (self->Poll() != 0) {
      return;
    }

    /* 调试镜像同步 / Refresh the debug mirror. */
    std::memcpy(MPU6050_data.gyro, self->gyro_.data(), sizeof(MPU6050_data.gyro));
    std::memcpy(MPU6050_data.accl, self->accl_.data(), sizeof(MPU6050_data.accl));
    std::memcpy(MPU6050_data.euler, self->euler_.data(), sizeof(MPU6050_data.euler));

    const auto TIMESTAMP = LibXR::Timebase::GetMicroseconds();
    self->topic_gyro_.Publish(self->gyro_, TIMESTAMP);
    self->topic_accl_.Publish(self->accl_, TIMESTAMP);
    self->topic_euler_.Publish(self->euler_, TIMESTAMP);
  }

  void OnMonitor() override {
    if (!gyro_.allFinite() || !accl_.allFinite() || !euler_.allFinite()) {
      XR_LOG_WARN("MPU6050: NaN/Inf detected. euler: %f %f %f", euler_.x(), euler_.y(),
                  euler_.z());
    }
  }

  /**
   * @brief 把浮点量按 scale 定标为整数（四舍五入，远离零）/ Scale a float value into an
   *        integer with round-half-away-from-zero.
   * @param value 待定标的浮点量 / Float value to scale.
   * @param scale 定标倍数 / Scale factor.
   * @return 定标后的整数 / Scaled integer.
   */
  static int ScaleToInt(float value, int scale) {
    const float SCALED = value * static_cast<float>(scale);
    return static_cast<int>(SCALED >= 0.0F ? SCALED + 0.5F : SCALED - 0.5F);
  }

  /**
   * @brief RamFS 调试命令 / RamFS debug command.
   * @param self 模块实例 / Module instance.
   * @param argc 参数个数 / Argument count.
   * @param argv 参数列表 / Argument vector.
   * @return 0 成功，-1 参数错误 / 0 on success, -1 on bad arguments.
   */
  static int CommandFunc(MPU6050* self, int argc, char** argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  whoami                  - Read the WHO_AM_I register.\r\n">();
      LibXR::STDIO::Printf<
          "  show [duration_ms] [interval_ms] - Print data periodically.\r\n">();
      LibXR::STDIO::Printf<
          "     Values are scaled integers: euler x100, gyro x10000, accl "
          "x1000.\r\n">();
    } else if (argc == 2) {
      if (strcmp(argv[1], "whoami") == 0) {
        LibXR::STDIO::Printf<"WHO_AM_I = 0x%02X\r\n">(
            mpu6050_read_reg(MPU_DEVICE_ID_REG));
      } else {
        LibXR::STDIO::Printf<"Error: unknown subcommand.\r\n">();
        return -1;
      }
    } else if (argc == 4) {
      if (strcmp(argv[1], "show") == 0) {
        int time = std::atoi(argv[2]);
        int interval = std::atoi(argv[3]);
        if (interval < 1) {
          interval = 1;
        }
        while (time > 0) {
          LibXR::STDIO::Printf<
              "Euler(deg*100): %+7d %+7d %+7d | Gyro(rad/s*1e4): %+7d %+7d "
              "%+7d | Accl(m/s^2*1e3): %+7d %+7d %+7d\r\n">(
              ScaleToInt(self->euler_.x(), K_EULER_SCALE),
              ScaleToInt(self->euler_.y(), K_EULER_SCALE),
              ScaleToInt(self->euler_.z(), K_EULER_SCALE),
              ScaleToInt(self->gyro_.x(), K_GYRO_SCALE),
              ScaleToInt(self->gyro_.y(), K_GYRO_SCALE),
              ScaleToInt(self->gyro_.z(), K_GYRO_SCALE),
              ScaleToInt(self->accl_.x(), K_ACCL_SCALE),
              ScaleToInt(self->accl_.y(), K_ACCL_SCALE),
              ScaleToInt(self->accl_.z(), K_ACCL_SCALE));
          LibXR::Thread::Sleep(interval);
          time -= interval;
        }
      } else {
        LibXR::STDIO::Printf<"Error: unknown subcommand.\r\n">();
        return -1;
      }
    } else {
      LibXR::STDIO::Printf<"Error: invalid arguments.\r\n">();
      return -1;
    }

    return 0;
  }

 private:
  Vec3 gyro_ = Vec3::Zero();
  Vec3 accl_ = Vec3::Zero();
  Vec3 euler_ = Vec3::Zero();

  LibXR::Topic topic_gyro_;
  LibXR::Topic topic_accl_;
  LibXR::Topic topic_euler_;

  LibXR::I2C* i2c_;
  LibXR::Timer::TimerHandle timer_handle_;
  LibXR::RamFS::File cmd_file_;
};
