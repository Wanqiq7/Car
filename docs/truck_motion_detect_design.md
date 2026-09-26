# 货车行驶状态检测（启动/停止判别）方案设计

- **工程**：Clock（STM32F103C8T6 · XRobot + LibXR · 裸机 `SYSTEM=None`）
- **日期**：2026-09-26
- **状态**：设计评审稿（未实施）
- **参考**：《STM32+MPU6050 计步器从驱动到算法：原理、滤波与参数调优》（CSDN，下称"计步文"）

---

## 0. 结论先行

1. **选定方案**：在 MPU6050 模块之**上**新建本地模块 `TruckMotion`，订阅现有 100 Hz Topic（零硬件改动），以"加速度幅值能量 + 陀螺能量 + 持续偏航"三路证据合成行驶判据，经**滞回状态机**输出二值状态。
2. **现有代码零改动**：不修改 `Modules/MPU6050` 任何文件；唯一改动是在 `User/xrobot.yaml` 追加一个实例（本地模块登记约定，见 §6.2）。
3. **核心物理依据**（算法为什么能区分）：
   - 怠速振动为**高频小幅**（发动机基频 >20 Hz）→ 被 f_c ≈ 3 Hz 一阶低通抑制，静止特征 σ_a 低；
   - 行驶时路面起伏 / 加减速为**低频大幅能量**（0.5~3 Hz）→ 落在低通通带内，σ_a 显著升高；
   - 匀速巡航平滑路面可能"能量塌陷" → 用**持续偏航/角速度**作第三路兜底证据。
4. **资源预算**：Flash 估 +1.5~2.5 KB（当前余量 6 248 B = 9.53%，构建后实测确认）；RAM < 200 B（全递推、无环形缓冲）。
5. **调参方法继承计步文 §4.2**：串口 CSV 采集 → PC 绘图量分布 → 阈值取分布间隔中点，**不盲调**。

---

## 1. 需求与状态定义

| 项 | 定义 |
|---|---|
| 输入 | 车载 MPU6050（安装方向任意，随车身振动；DMP 已出 100 Hz 物理量） |
| 输出 | 二值状态 `STOPPED / MOVING` + 状态切换事件（Topic 发布） |
| 时延要求（建议，可配） | 起步判"行驶" ≤ 3 s；停车判"停止" ≤ 10 s |
| 明确不做 | 不估速度/里程（积分漂移不可控）；不区分"怠速"与"熄火"（二者都是 STOPPED） |
| 失效安全 | 采样中断 → 特征冻结、状态保持、不误翻转（EMA 递推天然容忍丢样本） |

---

## 2. 现有系统盘点（数据源，零改动直接复用）

### 2.1 Topic 数据源

| Topic | 内容 | 单位 | 量程/标度 | 速率 | 出处 |
|---|---|---|---|---|---|
| `mpu6050_accl` | 三轴比力（含重力） | m/s² | **±2g**（16 384 LSB/g） | 100 Hz | `Modules/MPU6050/MPU6050.hpp:76-77,197-201` |
| `mpu6050_gyro` | 三轴角速度 | rad/s | ±2 000 dps（16.4 LSB/dps） | 100 Hz | `MPU6050.hpp:72-76,191-195` |
| `mpu6050_euler` | 俯仰/横滚/偏航 | deg | — | 100 Hz | `MPU6050.hpp:212-220` |

- 发布点：`MPU6050::SampleTaskFun`（`MPU6050.hpp:228-246`），由 `LibXR::Timer` 周期任务驱动，周期 `sample_period_ms: 10`（`User/xrobot.yaml:26`）。
- 订阅 API：`LibXR::Topic::FindOrCreate<Vec3>(name)`（`LibXR/src/middleware/message/topic.hpp:365-396`）+ `Topic::RegisterCallback(Callback&)`（`topic.hpp:304`），回调经 `Topic::Callback::Create(fun, this)` 构造（`subscriber/callback.hpp:496-505`）。
- **执行上下文**：回调在发布者（MPU6050 的定时器任务）上下文同步执行；`SYSTEM=None` 裸机单上下文 → 无并发、无锁需求，回调内**禁止阻塞/Printf**。

### 2.2 硬约束

| 约束 | 数值/规则 | 影响 |
|---|---|---|
| Flash | 59 288 / 65 536 B（90.47%），余量 6 248 B | 新模块必须 < ~3 KB；打印走定标整数 |
| 加速度量程 | ±2 g（`mpu_init()` 固定，DMP 库硬编码） | 大冲击削顶；本方案用方差特征，天然容忍 |
| 数据率 | DMP 固定 100 Hz，勿改 `sample_period_ms` | 特征时间常数按 10 ms 样本间隔设计 |
| 构建约定 | 本地模块只登记 `User/xrobot.yaml`，禁入 `Modules/modules.yaml` | 见 §6.2 |

---

## 3. 计步文方法论 → 本方案迁移

| 计步文技术 | 计步场景 | 本方案迁移 |
|---|---|---|
| 合成矢量幅值 `‖a‖=√(ax²+ay²+az²)`（§2.1） | 姿态无关的步态标量 | **直接沿用**：行驶能量特征与安装方向无关 |
| 重力直流偏置 ~1 g 需扣除（§2.1） | 滤波前去直流 | EMA 慢基线 `base`，`dev = ‖a‖ − base`（τ≈2 s，自动适应坡道停车/温漂） |
| 一阶低通 α=0.4@50 Hz（§2.2） | 压高频噪声、保留步态 1~2.5 Hz | α=0.2@100 Hz（f_c≈3.2 Hz）：**通带保路面事件、阻带压发动机怠速振动**——这正是区分"怠速/行驶"的关键 |
| 峰值检测 + 动态阈值 + 时间窗（§2.3） | 排除单次冲击误计步 | 滞回状态机 T_START=2 s / T_STOP=8 s：排除开关门、装卸、人碰等瞬态 |
| 动态基线 `baseline` 慢跟随（§2.3 代码） | 走/跑幅度差异大 | 阈值 = K × 怠速噪声底（自校准）+ 绝对下限钳位（§5.3） |
| 幅度硬上限 2.5 g 判冲击（§4.3） | 上下楼大冲击 | ±2 g 量程天然削顶 → σ 特征有界（局限已记录，§7-3） |
| 轴方差选主轴（§4.3） | 持握方向不定 | 全矢量 ‖a‖/‖ω‖ 特征天然姿态无关，无需选轴 |
| 陀螺仪区分"原地踏步"（§4.3） | 加速度计盲区 | 陀螺能量 + 持续偏航 = 匀速巡航兜底证据 |
| 串口 CSV → PC matplotlib 调参（§4.2） | "阈值 = 峰谷间隔中点，不盲调" | RamFS `truck log` 输出定标整数 CSV，同流程（§8.1） |
| 验证：分组测试 + 误差率 ±5%（§5） | 计步误差 | 路测矩阵 + 混淆矩阵 + 切换时延指标（§8.2） |

---

## 4. 备选方案对比

| 方案 | 思路 | 结论 |
|---|---|---|
| A. 原始幅值固定阈值 | `‖‖a‖−g‖ > TH` 即判行驶 | **弃**：怠速振动与路况差异使其要么误报要么漏报 |
| B. FFT 频谱分析 | 区分怠速窄带与行驶宽带 | **弃**：RAM/CPU 超出 F103 64 KB 预算，收益不成比例 |
| **C. 递推特征 + 滞回状态机（选定）** | EMA 特征（O(1) 内存）+ 三路证据 + 滞回 | O(1) RAM、无缓冲、Flash 可承受、可调参可验证 |
| D. 加速度积分估速度 | 积分 `dev` 得速度判行驶 | **弃**：漂移不可控，仅起步瞬间有意义；不纳入本期 |

---

## 5. 算法设计

### 5.1 特征链（每样本 10 ms，全递推 O(1)）

```
── 加速度支路 ──────────────────────────────────────────────
mag  = ‖accl‖                          # 姿态无关标量（含重力直流）
base += K_BASE · (mag − base)          # 重力基线 EMA，τ≈2 s
dev  = mag − base                      # 去直流
devf += K_LPF · (dev − devf)           # 一阶低通 f_c≈3.2 Hz（压怠速高频）
e    += K_E · (devf² − e)              # 短时能量 EMA，τ≈0.5 s
σ_a  = √e                              # 行驶主特征（m/s²）

── 陀螺支路 ────────────────────────────────────────────────
w    = ‖gyro‖
wl   = max(w − w_bias, 0)              # 扣校准期测得的零偏
wf   += K_LPF · (wl − wf)              # 同低通
m_w  += K_E · (wf − wf)                # 角速度能量特征（rad/s）

── 持续偏航判据（匀速巡航兜底） ─────────────────────────────
yaw_ms = (wf > TH_YAW) ? yaw_ms+10 : 0
```

设计依据：怠速振动（发动机 >20 Hz）被 3 Hz 低通衰减一个量级以上；路面起伏/加减速（0.5~3 Hz）落在通带。陀螺支路同理——怠速对陀螺几乎无低频贡献，**陀螺能量是"怠速 vs 行驶"区分度最高的特征**。

### 5.2 证据合成与滞回判决

```
evidence = (σ_a > TH_A) ∨ (m_w > TH_W) ∨ (yaw_ms ≥ 1000)

if state == STOPPED:
    t_run  = evidence ? t_run+1 : 0        # 按样本计数（×10 ms）
    if t_run ≥ T_START/10ms: state = MOVING; 发布切换事件
else:  # MOVING
    t_idle = evidence ? 0 : t_idle+1
    if t_idle ≥ T_STOP/10ms:  state = STOPPED; 发布切换事件
```

- **滞回不对称是刻意的**：判"行驶"要求 2 s 持续证据（排除关门/装卸冲击）；判"停止"要求 8 s 持续安静（排除巡航中短暂平整路段导致的假停止）。
- 状态切换时发布 Topic `truck_motion`；特征每样本更新但不逐样本发布（省总线）。

### 5.3 自校准与降级

| 项 | 设计 |
|---|---|
| 触发 | 上电自动一次（状态 `CALIBRATING`）；RamFS `truck calib` 可手动重校 |
| 校准条件（SOP） | **发动机怠速、车辆静止、持续 10 s** —— 校准的噪声底即"停止态"的真实噪声底 |
| 校准输出 | σ_idle_a、σ_idle_w（噪声底）、w_bias（陀螺零偏） |
| 阈值生成 | `TH_A = clamp(K_TH·σ_idle_a, TH_A_MIN, TH_A_MAX)`；`TH_W = clamp(K_TH·σ_idle_w, TH_W_MIN, TH_W_MAX)`，K_TH=6 |
| 降级模式 | 未校准/校准条件不满足（如熄火上电，噪声底偏小）→ 直接用绝对下限 TH_A_MIN/TH_W_MIN，功能可用、灵敏度保守 |

### 5.4 参数初值表

| 参数 | 初值 | 含义 | 备注 |
|---|---|---|---|
| `K_BASE` | 0.005 | 基线 EMA 系数 | τ≈2 s @100 Hz |
| `K_LPF` | 0.20 | 低通系数 | f_c ≈ 3.2 Hz @100 Hz |
| `K_E` | 0.02 | 能量 EMA 系数 | τ≈0.5 s |
| `K_TH` | 6 | 阈值 = K × 噪声底 | 校准后生效 |
| `TH_A_MIN / MAX` | 0.10 / 0.50 m/s² | 加速度阈值钳位 | 0.10 m/s² ≈ 10 mg |
| `TH_W_MIN / MAX` | 0.01 / 0.05 rad/s | 陀螺阈值钳位 | 0.01 rad/s ≈ 0.57°/s |
| `TH_YAW` | 0.05 rad/s | 持续偏航门限 | ≈ 2.9°/s |
| `T_YAW` | 1.0 s | 偏航持续时长 | |
| `T_START` | 2 s | 判"行驶"防抖 | 计步文时间窗思想 |
| `T_STOP` | 8 s | 判"停止"防抖 | 抗巡航能量塌陷 |
| `T_CAL` | 10 s | 校准窗长 | |

> 全部阈值经 §8.1 的 CSV 采集流程用实测数据复核修正——**以上仅为上电首跑值，不是定稿值**。

---

## 6. 软件集成设计

### 6.1 模块骨架（`Modules/TruckMotion/TruckMotion.hpp`，节选）

```cpp
/* === MODULE MANIFEST V2 ===
module_description: 货车行驶状态检测（MPU6050 数据消费方）
constructor_args:
  - accl_topic_name: "mpu6050_accl"
  - gyro_topic_name: "mpu6050_gyro"
  - output_topic_name: "truck_motion"
  - calib_ms: 10000
required_hardware: ramfs
depends: []
=== END MANIFEST === */
class TruckMotion : public LibXR::Application {
 public:
  using Vec3 = Eigen::Matrix<float, 3, 1>;
  enum class State : uint8_t { STOPPED = 0, MOVING = 1 };

  /* Ozone 调试镜像（约定同 MPU6050::MPU6050_data，扁平 struct） */
  inline static struct {
    float sigma_a, mean_w;        // 特征
    uint32_t yaw_ms, evidence_ms; // 证据累计
    uint8_t state, calibrated;    // 状态 / 校准标志
    uint32_t sample_count;        // 入口计数（诊断调度）
  } TruckMotion_data{};

  TruckMotion(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const char* accl_topic, const char* gyro_topic,
              const char* out_topic, uint32_t calib_ms)
      : topic_out_(LibXR::Topic::CreateTopic<Out>(out_topic)),
        cmd_file_(LibXR::RamFS::CreateFile("truck", CommandFunc, this)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
    LibXR::Topic(LibXR::Topic::FindOrCreate<Vec3>(accl_topic))
        .RegisterCallback(cb_accl_);   // cb_accl_ = Callback::Create(OnAccl, this)
    LibXR::Topic(LibXR::Topic::FindOrCreate<Vec3>(gyro_topic))
        .RegisterCallback(cb_gyro_);
    /* 启动校准窗口（样本计数驱动，无需额外 Timer 任务） */
  }

 private:
  static void OnAccl(bool in_isr, TruckMotion* self, const Vec3& a); // 加速度特征递推
  static void OnGyro(bool in_isr, TruckMotion* self, const Vec3& w); // 陀螺特征递推
  void Step();   // 证据合成 + 滞回判决 + 切换发布（每样本一次，~20 flops + 1 sqrtf）

  /* 特征状态（全部 float 标量，< 100 B） */
  float base_ = 1.0F * 9.80665F, devf_ = 0, e_ = 0;
  float w_bias_ = 0, wf_ = 0, m_w_ = 0;
  uint32_t yaw_ms_ = 0, t_run_ = 0, t_idle_ = 0, cal_cnt_ = 0;
  State state_ = State::STOPPED;
  /* 阈值成员（校准后写入）…、Topic 句柄、Callback 块、RamFS File … */
};
```

要点：

- **无自有定时器**：特征递推与判决直接挂在 MPU6050 的 100 Hz 发布回调里，避免第二个定时任务的相位漂移与重复采样。
- 回调在 `SampleTaskFun` 上下文同步执行（topic.hpp:682-707 发布路径直接分发），计算量约 20 次浮点运算 + 1 次 `sqrtf`/样本，@72 MHz 开销可忽略；**回调内不做 Printf**。
- 输出 payload：`struct Out { uint8_t state; uint8_t calibrated; float sigma_a; float mean_w; }`（trivially copyable，满足 Topic 契约）。

### 6.2 构建与登记

1. 新建 `Modules/TruckMotion/CMakeLists.txt`（模板照抄 `Modules/MPU6050/CMakeLists.txt:1-12`，GLOB `*.cpp` 进 `xr`）。
2. `User/xrobot.yaml` **在 `MPU6050_0` 之后**追加实例（列表顺序 = 生成代码中的构造顺序，必须保证 Topic 已创建）：

   ```yaml
   - id: TruckMotion_0
     name: TruckMotion
     constructor_args:
       accl_topic_name: mpu6050_accl
       gyro_topic_name: mpu6050_gyro
       output_topic_name: truck_motion
       calib_ms: 10000
   ```

3. **不登记** `Modules/modules.yaml`（本地模块约定）；`cmake --build` 会经 `CONFIGURE_DEPENDS` 自动重跑 `xrobot_gen_main` 重新生成 `xrobot_main.hpp`。

### 6.3 调试设施

| 设施 | 设计 |
|---|---|
| RamFS 命令 `truck` | `status`（打印状态+特征定标整数）；`calib`（重校准）；`log <ms> [interval]`（CSV：`t, mag×1000, devf×1000, σ_a×1e4, m_w×1e4, state`，走 USART1 115200，参照 `mpu6050 show` 的定标整数风格 `MPU6050.hpp:81-86,262-265`） |
| Ozone 镜像 | Watched Data 只填 `TruckMotion::TruckMotion_data`（扁平 struct，约定同 `MPU6050_data`）；Program File 固定 `build/RelWithDebInfo/Clock.elf` |
| PC 侧 | 串口收 CSV → matplotlib 叠绘 raw/devf/σ_a 与阈值线（计步文 §4.2 同款脚本） |

### 6.4 资源预算

| 项 | 估算 | 依据 |
|---|---|---|
| Flash | +1.5~2.5 KB → 预计 92~94% | 浮点特征链（小）、Topic 订阅/发布、RamFS 命令与格式串；`sqrtf` 已在镜像（euler 计算用 `std::asin/atan2`）；打印定标整数不拉入浮点格式化之外的新路径 |
| RAM | < 200 B | 全标量递推状态；无环形缓冲（EMA 方案比 100 样本窗省 400 B）；回调块由 LibXR `new` 分配 |
| 验证 | 构建后以链接器报数为准 | 超预算时的裁剪顺序：砍 `log` 命令 → 砍 `calib` 命令 → 参数只留编译期常量 |

---

## 7. 风险与缓解

| # | 场景 | 风险 | 缓解 | 残余风险 |
|---|---|---|---|---|
| 1 | 匀速 + 平整路面（高速巡航） | 能量低于阈值 → 假"停止" | 持续偏航兜底 + `T_STOP` 拉长至 8 s + `TH_A_MIN` 下限 | **中**：需路测确认，必要时 `T_STOP` 提到 10~15 s 或降 `TH_A_MIN` |
| 2 | 校准条件与实际"停止"态不一致（怠速 vs 熄火） | 噪声底差一个量级 → 假"行驶" | 校准 SOP=怠速；绝对下限钳位；`truck calib` 重校 | 低 |
| 3 | 坑洞等大冲击 | ±2 g 削顶，σ 瞬间低估 | 削顶只影响单样本，EMA 平滑；量程改动涉及 MPU6050 模块，不在本期 | 低 |
| 4 | 安装方向任意 / 竖装 | yaw（ω_z）判据失效 | 主判据用全矢量 ‖ω‖，ω_z 仅可选增强 | 低 |
| 5 | 温漂 / 陀螺零偏漂移 | m_w 虚高 → 假"行驶" | 校准期测零偏扣除；加速度侧 EMA 基线慢跟踪 | 低 |
| 6 | FIFO 偶发空（Poll 非 0） | 特征时间轴拉伸 | EMA 递推天然容忍；计数按样本 ×10 ms | 低 |
| 7 | Flash 超预算 | 链接失败 | §6.4 裁剪顺序；构建实测 | 低（余量 6.1 KB） |
| 8 | 装卸货 / 开关门冲击 | 假"行驶" | 判"行驶"需 2 s 持续证据，且特征经 3 Hz 低通（冲击高频被滤） | 低 |

---

## 8. 验证方案

### 8.1 台架调参（先于路测，继承计步文 §4.2"不盲调"原则）

1. 固件烧录后，`truck log 60000 20` 经 USART1 采 60 s CSV；
2. 覆盖工况：熄火静止 / **怠速静止** / 手持模拟颠簸；
3. PC matplotlib 绘 mag/devf/σ_a 曲线 + 噪声底统计直方图；
4. 按实测分布定阈值：怠速 σ 分布上界 ×6 与绝对下限取大者；与计步文"阈值 = 峰谷间隔中点"准则同源。

### 8.2 路测矩阵（验收线）

| 工况 | 时长 | 期望 | 通过线 |
|---|---|---|---|
| 怠速静止 | ≥ 30 min | 恒 STOPPED | **0 次误切换** |
| 起步加速 | — | ≤ 3 s 切 MOVING | 时延 ≤ 3 s |
| 市区行驶 | ≥ 30 min | 恒 MOVING | 0 次假停止 |
| 高速巡航 | ≥ 30 min | 恒 MOVING | 0 次假停止（风险 #1 专项） |
| 停车（滑行→静止） | — | ≤ 10 s 切 STOPPED | 时延 ≤ 10 s |
| 熄火静止 | ≥ 1 h | 恒 STOPPED | 0 次误切换 |

### 8.3 反向验证（negative tests，确认机制真的在起作用）

1. **阈值失效注入**：`TH_A` 人为调低至怠速噪声底以下 → **必须**观察到假"行驶"（证明阈值机制有效、特征对怠速敏感，即系统能复现原始失效模式）；
2. **降级模式**：熄火状态上电（校准噪声底偏小）→ 确认走绝对下限仍正常判别，不发散、不振荡；
3. **数据源中断**：运行中模拟 I2C 断链（拔线）→ `sample_count` 停增、特征冻结、状态保持原值不翻转。

---

## 9. 实施清单

1. 新建 `Modules/TruckMotion/{TruckMotion.hpp, CMakeLists.txt}`；
2. `User/xrobot.yaml` 追加 `TruckMotion_0`（`MPU6050_0` 之后）；
3. `tools/Windows/build.ps1` 构建 → 确认 Flash 实际占用（预期 ≤ 94%）；
4. RamFS `truck log` + PC 脚本台架采集，定阈值初稿；
5. 路测矩阵 + 反向验证（§8.2/§8.3），参数定稿；
6. 补 `Modules/TruckMotion/README.md`，记录最终参数与路测数据。

> 实施约束提醒：新增文件仅 `Modules/TruckMotion/` 两个；除 `User/xrobot.yaml` 追加实例外，**不新增、不删除、不修改任何现有文件**（含 `Modules/MPU6050` 全部文件——其中 `inv_mpu.c/.h`、`MPU6050.c/.h` 为 GBK 编码，本方案不触碰）。
