#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 货车行驶状态检测（消费 MPU6050 Topic，输出 truck_motion 状态）
constructor_args:
  - accl_topic_name: "mpu6050_accl"
  - gyro_topic_name: "mpu6050_gyro"
  - output_topic_name: "truck_motion"
  - calib_ms: 10000
template_args: []
required_hardware: ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "message.hpp"

/**
 * @brief 货车行驶状态检测模块（启动/停止判别）。
 *
 * @details
 * 订阅 MPU6050 模块发布的 100 Hz Topic，用递推特征链合成"行驶证据"，经不对称
 * 滞回状态机输出二值状态到 `truck_motion` Topic。**本模块没有自有定时任务**：
 * 特征递推与判决全部挂在 MPU6050 `SampleTaskFun` 的发布回调内同步执行
 * （该上下文由 MPU6050 的 LibXR Timer 任务驱动），因此实例必须登记在
 * `User/xrobot.yaml` 中 `MPU6050_0` 之后。
 *
 * 数据流（每样本 10 ms，全部 O(1) 递推，无环形缓冲）：
 * ```
 * mpu6050_accl ─► ‖a‖ ─► EMA重力基线(τ≈2s) ─► 一阶低通(f_c≈3.2Hz) ─► EMA能量 σ_a ─┐
 * mpu6050_gyro ─► ‖ω‖−零偏 ─► 一阶低通 ─► EMA均值 m_w ──────────────────────────┤─► 证据OR ─► 滞回状态机
 * 持续偏航（wf > TH_YAW 持续 1 s，匀速巡航兜底）────────────────────────────────┘
 * ```
 *
 * 物理依据：发动机怠速振动为高频小幅信号，被 3 Hz 低通抑制；行驶时路面起伏/
 * 加减速为低频（0.5~3 Hz）大幅能量，落在通带内。陀螺支路对"怠速 vs 行驶"的
 * 区分度最高；匀速平整路面可能能量塌陷，由持续偏航判据兜底。
 *
 * 判决（不对称滞回，参照计步算法"峰值 + 动态阈值 + 时间窗"思想）：
 * - STOPPED → MOVING：证据连续 2 s（抗装卸、开关门等单次冲击）；
 * - MOVING → STOPPED：无证据连续 8 s（抗巡航平整路段的能量塌陷）。
 *
 * 自校准：上电（或 `truck calib` 命令）后累计 `calib_ms` 的静止样本，统计
 * ‖a‖/‖ω‖ 的均值与方差：均值作为重力基线与陀螺零偏，6×标准差经上下限钳位后
 * 作为阈值。**校准 SOP：发动机怠速、车辆静止**（怠速噪声底即"停止态"的真实
 * 噪声底）。未校准/校准条件不满足时使用绝对下限（降级模式，灵敏度保守）。
 *
 * @note 本工程 `SYSTEM=None`（裸机协作式）：回调在 MPU6050 的 Timer 任务上下文
 *       同步执行，**回调内不做任何阻塞/打印**；状态切换事件在同一上下文发布到
 *       `truck_motion`（与 mpu6050_accl 属不同 Topic，发布锁互不影响）。
 *       阈值仅为上电首跑值，定稿须按设计文档 §8.1 用 `truck log` CSV 实测调参。
 */
class TruckMotion : public LibXR::Application {
 public:
  using Vec3 = Eigen::Matrix<float, 3, 1>;

  /** 状态输出 payload（trivially copyable，满足 Topic 契约）。 */
  struct Out {
    uint8_t state;       ///< State 枚举值。
    uint8_t calibrated;  ///< 自校准完成标志。
    float sigma_a;       ///< 加速度能量特征（m/s²）。
    float mean_w;        ///< 角速度能量特征（rad/s）。
  };

  enum class State : uint8_t { CALIBRATING = 0, STOPPED = 1, MOVING = 2 };

  /* ── 算法参数（编译期常量；初值依据设计文档 §5.4，定稿以实测调参为准）─────── */
  static constexpr float K_BASE = 0.005F;  ///< 重力基线 EMA 系数（τ≈2 s）。
  static constexpr float K_LPF = 0.20F;    ///< 一阶低通系数（f_c≈3.2 Hz @100 Hz）。
  static constexpr float K_E = 0.02F;      ///< 能量 EMA 系数（τ≈0.5 s）。
  static constexpr float K_TH = 6.0F;      ///< 阈值 = K_TH × 校准噪声底。
  static constexpr float TH_A_MIN = 0.10F;  ///< σ_a 阈值下限（m/s²，≈10 mg）。
  static constexpr float TH_A_MAX = 0.50F;  ///< σ_a 阈值上限（m/s²）。
  static constexpr float TH_W_MIN = 0.01F;  ///< m_w 阈值下限（rad/s，≈0.57°/s）。
  static constexpr float TH_W_MAX = 0.05F;  ///< m_w 阈值上限（rad/s）。
  static constexpr float TH_YAW = 0.05F;   ///< 持续偏航门限（rad/s，≈2.9°/s）。
  static constexpr uint32_t T_YAW_MS = 1000;       ///< 偏航持续时长（ms）。
  static constexpr uint32_t T_START_SAMPLES = 200;  ///< 判行驶防抖（2 s @100 Hz）。
  static constexpr uint32_t T_STOP_SAMPLES = 800;   ///< 判停止防抖（8 s @100 Hz）。

  /* 打印定标倍数：幅值/加速度阈值 ×1e3，角速度/σ_a ×1e4（不经浮点格式化路径）。 */
  static constexpr int K_SCALE_ACCL = 1000;
  static constexpr int K_SCALE_W = 10000;

  /* ── Ozone 调试镜像（约定同 MPU6050::MPU6050_data，扁平 struct）────────────
     Watched Data 只填 `TruckMotion::TruckMotion_data`；`sample_count` 在 accl
     回调入口自增，用于区分"任务未调度"与"特征全 0"。 */
  inline static struct {
    float sigma_a;          ///< 加速度能量特征（m/s²）。
    float mean_w;           ///< 角速度能量特征（rad/s）。
    float base;             ///< 当前重力基线（m/s²）。
    uint32_t yaw_ms;        ///< 偏航持续计时（ms）。
    uint32_t sample_count;  ///< 回调入口计数（按 mpu6050_accl 计）。
    uint8_t state;          ///< State 枚举值。
    uint8_t calibrated;     ///< 校准完成标志。
  } TruckMotion_data{};

  /**
   * @brief 构造模块并订阅 MPU6050 数据源。
   *
   * @param hw 硬件容器（需要 ramfs）。
   * @param app 应用管理器。
   * @param accl_topic_name 加速度 Topic 名（m/s²，须与 MPU6050 实例配置一致）。
   * @param gyro_topic_name 角速度 Topic 名（rad/s）。
   * @param output_topic_name 状态输出 Topic 名。
   * @param calib_ms 上电自校准时长（ms），按 100 Hz 折算样本数。
   */
  TruckMotion(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const char* accl_topic_name, const char* gyro_topic_name,
              const char* output_topic_name, uint32_t calib_ms)
      : topic_out_(LibXR::Topic::CreateTopic<Out>(output_topic_name)),
        cal_n_target_(calib_ms / 10),  /* 100 Hz（sample_period_ms=10） */
        cb_accl_(LibXR::Topic::Callback::Create(OnAccl, this)),
        cb_gyro_(LibXR::Topic::Callback::Create(OnGyro, this)),
        cmd_file_(LibXR::RamFS::CreateFile("truck", CommandFunc, this)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    /* 依赖 MPU6050_0 先构造（xrobot.yaml 中实例顺序即构造顺序）。 */
    LibXR::Topic accl(LibXR::Topic::FindOrCreate<Vec3>(accl_topic_name));
    accl.RegisterCallback(cb_accl_);
    LibXR::Topic gyro(LibXR::Topic::FindOrCreate<Vec3>(gyro_topic_name));
    gyro.RegisterCallback(cb_gyro_);

    PublishState();
    XR_LOG_INFO("TruckMotion: subscribed, calibrating...");
  }

  /* 静默巡检：特征出现 NaN/Inf 时仅累加计数（`truck status` 可见），不打印——
     LibXR 打印按格式签名实例化模板，新增 %f/%u/%s 变体会各复制一套 Executor
     （约 1.5~2 KB），本工程 Flash 余量极紧（设计文档 §6.4）。 */
  void OnMonitor() override {
    if (!std::isfinite(sigma_a_) || !std::isfinite(m_w_) ||
        !std::isfinite(base_)) {
      if (nan_cnt_ < 255U) {
        ++nan_cnt_;
      }
    }
  }

  /**
   * @brief 把浮点量按 scale 定标为整数（四舍五入，远离零）。
   */
  static int ScaleToInt(float value, int scale) {
    const float SCALED = value * static_cast<float>(scale);
    return static_cast<int>(SCALED >= 0.0F ? SCALED + 0.5F : SCALED - 0.5F);
  }

  /**
   * @brief RamFS 调试命令 `truck`（全部 %d 定标整数，复用既有打印模板实例）。
   *
   * - `truck status`：state（0=CALIBRATING 1=STOPPED 2=MOVING）、校准/异常计数、
   *   特征与阈值（sigma/m_w ×1e4，th_a ×1e3）。
   * - `truck calib`：请求重新校准（在下一采样周期生效）。
   * - `truck log <duration_ms> <interval_ms>`：CSV 输出
   *   `t_ms, mag*1e3, devf*1e3, sigma_a*1e4, m_w*1e4, state`，供 PC 端
   *   matplotlib 调参（对应设计文档 §8.1，禁止盲调）。
   */
  static int CommandFunc(TruckMotion* self, int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "status") == 0) {
      LibXR::STDIO::Printf<
          "state:%d cal:%d nan:%d sigma_e4:%d mw_e4:%d th_a_e3:%d "
          "th_w_e4:%d samples:%d\r\n">(
          static_cast<int>(self->state_), self->calibrated_ ? 1 : 0,
          static_cast<int>(self->nan_cnt_),
          ScaleToInt(self->sigma_a_, K_SCALE_W),
          ScaleToInt(self->m_w_, K_SCALE_W),
          ScaleToInt(self->th_a_, K_SCALE_ACCL),
          ScaleToInt(self->th_w_, K_SCALE_W),
          static_cast<int>(TruckMotion_data.sample_count));
    } else if (argc == 2 && strcmp(argv[1], "calib") == 0) {
      self->cal_req_ = true;
      LibXR::STDIO::Printf<
          "calibration requested (engine idling, vehicle stationary)\r\n">();
    } else if (argc == 4 && strcmp(argv[1], "log") == 0) {
      int time = std::atoi(argv[2]);
      int interval = std::atoi(argv[3]);
      if (interval < 1) {
        interval = 1;
      }
      LibXR::STDIO::Printf<"t_ms,mag_e3,devf_e3,sigma_e4,mw_e4,state\r\n">();
      while (time > 0) {
        LibXR::STDIO::Printf<"%d,%d,%d,%d,%d,%d\r\n">(
            static_cast<int>(LibXR::Timebase::GetMicroseconds() / 1000U),
            ScaleToInt(self->mag_, K_SCALE_ACCL),
            ScaleToInt(self->devf_, K_SCALE_ACCL),
            ScaleToInt(self->sigma_a_, K_SCALE_W),
            ScaleToInt(self->m_w_, K_SCALE_W),
            static_cast<int>(self->state_));
        LibXR::Thread::Sleep(interval);
        time -= interval;
      }
    } else {
      LibXR::STDIO::Printf<
          "Usage: truck status | truck calib | truck log <ms> <interval_ms> "
          "(state: 0=CALIBRATING 1=STOPPED 2=MOVING)\r\n">();
      return -1;
    }
    return 0;
  }

 private:
  /* ── 回调：在 MPU6050::SampleTaskFun 上下文按 gyro → accl 顺序同步执行 ───── */

  static void OnGyro(bool, TruckMotion* self, const Vec3& w) {
    const float WNORM = w.norm();
    if (!std::isfinite(WNORM)) {
      return;
    }
    self->w_ = WNORM;

    if (!self->calibrated_) {
      /* 校准期：累积原始统计量（均值/方差），不做特征递推。 */
      self->cal_w_sum_ += WNORM;
      self->cal_w_sq_ += WNORM * WNORM;
      return;
    }

    const float WL = WNORM - self->w_bias_;
    const float WL_POS = WL > 0.0F ? WL : 0.0F;
    self->wf_ += K_LPF * (WL_POS - self->wf_);
    self->m_w_ += K_E * (self->wf_ - self->m_w_);
  }

  static void OnAccl(bool, TruckMotion* self, const Vec3& a) {
    ++TruckMotion_data.sample_count;

    if (self->cal_req_) {
      self->cal_req_ = false;
      self->StartCalibration();
    }

    const float MAG = a.norm();
    if (!std::isfinite(MAG)) {
      return;
    }
    self->mag_ = MAG;
    self->base_ += K_BASE * (MAG - self->base_);

    if (!self->calibrated_) {
      self->cal_mag_sum_ += MAG;
      self->cal_mag_sq_ += MAG * MAG;
      if (++self->cal_n_ >= self->cal_n_target_) {
        self->FinishCalibration();
      }
      TruckMotion_data.base = self->base_;
      return;
    }

    /* 加速度特征链：去直流 → 低通 → 短时能量。 */
    const float DEV = MAG - self->base_;
    self->devf_ += K_LPF * (DEV - self->devf_);
    self->e_ += K_E * (self->devf_ * self->devf_ - self->e_);
    self->sigma_a_ = std::sqrt(self->e_);

    /* 持续偏航判据（样本周期 10 ms）。 */
    self->yaw_ms_ = (self->wf_ > TH_YAW) ? self->yaw_ms_ + 10U : 0U;

    self->UpdateState();

    /* 调试镜像同步。 */
    TruckMotion_data.sigma_a = self->sigma_a_;
    TruckMotion_data.mean_w = self->m_w_;
    TruckMotion_data.base = self->base_;
    TruckMotion_data.yaw_ms = self->yaw_ms_;
    TruckMotion_data.state = static_cast<uint8_t>(self->state_);
    TruckMotion_data.calibrated = self->calibrated_ ? 1U : 0U;
  }

  void StartCalibration() {
    cal_n_ = 0;
    cal_mag_sum_ = cal_mag_sq_ = 0.0F;
    cal_w_sum_ = cal_w_sq_ = 0.0F;
    devf_ = e_ = sigma_a_ = 0.0F;
    wf_ = m_w_ = 0.0F;
    t_run_ = t_idle_ = yaw_ms_ = 0U;
    calibrated_ = false;
    if (state_ != State::CALIBRATING) {
      state_ = State::CALIBRATING;
      PublishState();
    }
  }

  void FinishCalibration() {
    const float INV = 1.0F / static_cast<float>(cal_n_);
    const float MEAN_A = cal_mag_sum_ * INV;
    const float MEAN_W = cal_w_sum_ * INV;
    float var_a = cal_mag_sq_ * INV - MEAN_A * MEAN_A;
    float var_w = cal_w_sq_ * INV - MEAN_W * MEAN_W;
    var_a = var_a > 1e-6F ? var_a : 1e-6F;
    var_w = var_w > 1e-8F ? var_w : 1e-8F;

    base_ = MEAN_A;
    w_bias_ = MEAN_W;
    th_a_ = Clamp(K_TH * std::sqrt(var_a), TH_A_MIN, TH_A_MAX);
    th_w_ = Clamp(K_TH * std::sqrt(var_w), TH_W_MIN, TH_W_MAX);

    /* σ_a 以噪声底为种子，避免校准完成瞬间的假证据。 */
    e_ = var_a;
    sigma_a_ = std::sqrt(e_);
    devf_ = 0.0F;
    wf_ = m_w_ = 0.0F;
    t_run_ = t_idle_ = yaw_ms_ = 0U;
    calibrated_ = true;
    state_ = State::STOPPED;
    PublishState();
  }

  void UpdateState() {
    const bool EVIDENCE =
        (sigma_a_ > th_a_) || (m_w_ > th_w_) || (yaw_ms_ >= T_YAW_MS);
    bool changed = false;

    if (state_ == State::STOPPED) {
      t_run_ = EVIDENCE ? t_run_ + 1U : 0U;
      if (t_run_ >= T_START_SAMPLES) {
        state_ = State::MOVING;
        t_idle_ = 0U;
        changed = true;
      }
    } else if (state_ == State::MOVING) {
      t_idle_ = EVIDENCE ? 0U : t_idle_ + 1U;
      if (t_idle_ >= T_STOP_SAMPLES) {
        state_ = State::STOPPED;
        t_run_ = 0U;
        changed = true;
      }
    }

    if (changed) {
      PublishState();
    }
  }

  void PublishState() {
    out_.state = static_cast<uint8_t>(state_);
    out_.calibrated = calibrated_ ? 1U : 0U;
    out_.sigma_a = sigma_a_;
    out_.mean_w = m_w_;
    topic_out_.Publish(out_);
    TruckMotion_data.state = out_.state;
    TruckMotion_data.calibrated = out_.calibrated;
  }

  static float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  /* Topic / 订阅 / 命令（声明顺序 = 构造初始化顺序）。 */
  LibXR::Topic topic_out_;
  uint32_t cal_n_target_;
  LibXR::Topic::Callback cb_accl_;
  LibXR::Topic::Callback cb_gyro_;
  LibXR::RamFS::File cmd_file_;

  /* 加速度支路特征。 */
  float mag_ = 0.0F;
  float base_ = 9.80665F;  /* 重力初值（上电静态假定，校准后覆盖）。 */
  float devf_ = 0.0F;
  float e_ = 0.0F;
  float sigma_a_ = 0.0F;

  /* 陀螺支路特征。 */
  float w_ = 0.0F;
  float wf_ = 0.0F;
  float m_w_ = 0.0F;

  /* 校准状态。 */
  uint32_t cal_n_ = 0;
  float cal_mag_sum_ = 0.0F;
  float cal_mag_sq_ = 0.0F;
  float cal_w_sum_ = 0.0F;
  float cal_w_sq_ = 0.0F;
  float w_bias_ = 0.0F;
  float th_a_ = TH_A_MIN;
  float th_w_ = TH_W_MIN;
  bool calibrated_ = false;
  bool cal_req_ = false;

  /* 判决状态。 */
  State state_ = State::CALIBRATING;
  uint32_t t_run_ = 0;
  uint32_t t_idle_ = 0;
  uint32_t yaw_ms_ = 0;
  Out out_{};

  /* OnMonitor 静默巡检计数：NaN/Inf 出现次数（饱和于 255），`truck status` 可见。 */
  uint8_t nan_cnt_ = 0;
};
