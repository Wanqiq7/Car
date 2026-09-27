# TruckMotion — 货车行驶状态检测模块

订阅 `MPU6050` 模块的 100 Hz Topic，判别货车**启动/停止**，状态发布到
`truck_motion` Topic。设计文档与验证方案见
[`docs/truck_motion_detect_design.md`](../../docs/truck_motion_detect_design.md)。

## 数据流

```
mpu6050_accl ─► ‖a‖ ─► EMA重力基线 ─► LPF(f_c≈3.2Hz) ─► EMA能量 σ_a ─┐
mpu6050_gyro ─► ‖ω‖−零偏 ─► LPF ─► EMA均值 m_w ─────────────────────┤─► 证据OR ─► 滞回状态机
持续偏航（wf > TH_YAW 持续 1 s，匀速巡航兜底）────────────────────────┘
```

- **无自有定时任务**：全部计算在 MPU6050 的 100 Hz 发布回调内同步执行
  （O(1) 递推，无环形缓冲），实例必须排在 `MPU6050_0` 之后。
- 状态机：`CALIBRATING → STOPPED ⇄ MOVING`；判行驶需证据连续 2 s，
  判停止需无证据连续 8 s（不对称滞回）。
- 输出 payload：`{ state, calibrated, sigma_a, mean_w }`。

## 自校准

上电自动校准 `calib_ms`（默认 10 s）。**SOP：发动机怠速、车辆静止。**
校准得噪声底与陀螺零偏，阈值 = 6×噪声底并钳位
（σ_a ∈ [0.10, 0.50] m/s²，m_w ∈ [0.01, 0.05] rad/s）。
未校准/校准条件不满足时使用绝对下限（降级模式）。

## RamFS 命令（`truck`）

| 命令 | 说明 |
|---|---|
| `truck status` | 打印状态、特征与阈值（定标整数） |
| `truck calib` | 请求重新校准（下一采样周期生效） |
| `truck log <duration_ms> <interval_ms>` | CSV：`t_ms, mag*1e3, devf*1e3, sigma_a*1e4, m_w*1e4, state`，供 PC matplotlib 调参 |

## 调试

- Ozone Watched Data：`TruckMotion::TruckMotion_data`（扁平镜像，约定同
  `MPU6050::MPU6050_data`），含特征（sigma_a/mean_w/base）、阈值读视图
  （th_a/th_w）、状态与 `self` 实例指针；Program File 固定
  `build/RelWithDebInfo/Clock.elf`。
- **在线改阈值（免重烧）**：Ozone 中 halt → 展开 `self` → 编辑实例成员
  `th_a_`/`th_w_`（m/s²、rad/s）→ resume，立即生效。注意镜像里的 th_a/th_w
  只是读视图，改它们会被成员覆写。
- 定量调参走 `truck log` CSV（Ozone 只能定性观察与点值编辑）：
  阈值 = 怠速 σ 分布上界 ×6 与绝对下限取大，修改 `K_TH` / `TH_*_MIN` / `TH_*_MAX`
  常量后重新编译定稿。
- 反向验证：halt 下把 `th_a_` 改成 0.001（必现假 MOVING）或 5.0（恒 STOPPED），
  验证阈值机制有效。

## 参数速查

| 参数 | 初值 | 含义 |
|---|---|---|
| `K_BASE` | 0.005 | 重力基线 EMA（τ≈2 s） |
| `K_LPF` | 0.20 | 一阶低通（f_c≈3.2 Hz @100 Hz） |
| `K_E` | 0.02 | 能量 EMA（τ≈0.5 s） |
| `K_TH` | 6 | 阈值 = K × 噪声底 |
| `TH_A_MIN/MAX` | 0.10 / 0.50 m/s² | σ_a 阈值钳位 |
| `TH_W_MIN/MAX` | 0.01 / 0.05 rad/s | m_w 阈值钳位 |
| `TH_YAW` / `T_YAW_MS` | 0.05 rad/s / 1000 ms | 持续偏航判据 |
| `T_START_SAMPLES` / `T_STOP_SAMPLES` | 200 / 800 | 判行驶/判停止防抖 |
