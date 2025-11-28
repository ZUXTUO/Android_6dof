# 6DOF Android Demo

一个使用 Android 传感器与原生 EKF 融合的 6 自由度位姿可视化示例。应用通过 `GLSurfaceView` 渲染网格与方向轴，实时显示设备的空间位置与姿态，并绘制移动轨迹。

## 项目简介
- 实时读取加速度计、陀螺仪、磁力计、线性加速度与气压计数据。
- 原生 C++ 实现扩展卡尔曼滤波（EKF），包含预测、零速更新（ZUPT）、重力/磁力校正、气压高度更新。
- OpenGL ES 2.0 渲染：地面网格、正负方向轴（X/Z 分色）、相机视角环绕、轨迹线。

## 功能特性
- 位姿融合：支持 Acc/Gyro/Mag/Linear Acc/Barometer 的组合。
- 快速静止响应：ZUPT 与速度阻尼在停止后迅速归零，抑制“滑行”。
- 稳定性优化：线性加速度低通、磁力更新条件收紧、过程噪声调优、气压高度权重降低。
- 渲染视图：
  - 平面网格与地面方向轴（+X 红色，-X 深红；+Z 蓝色，-Z 深蓝；+Y 绿色）。
  - 轨迹记录：停止抖动后稳定追加，避免过密点。
  - 交互控制：单指拖动环绕，双指缩放。

## 架构与坐标系
- `app/src/main/java/com/olsc/t6dof/MainActivity.java`：UI、生命周期与手势；检查陀螺仪可用性；启动/停止传感器。
- `app/src/main/java/com/olsc/t6dof/sensors/SensorFusionManager.java`：注册传感器、回调上报到原生。
- `app/src/main/java/com/olsc/t6dof/sensors/NativeFusion.java`：JNI 封装，加载 `fusion` 原生库。
- `app/src/main/cpp/fusion.cpp`：JNI 实现、EKF 状态管理、传感器入口、门限与滤波。
- `app/src/main/cpp/ekf.cpp/.h`：EKF 预测/更新与状态矩阵。
- `app/src/main/java/com/olsc/t6dof/render/PoseRenderer.java`：OpenGL ES 2.0 渲染与相机控制。

坐标系约定：
- EKF 输出为世界坐标 ENU（East-North-Up），位姿格式 `[px, py, pz, qw, qx, qy, qz]`。
- 渲染使用 Y-Up；为对齐，渲染时对 `y/z` 做了映射并对四元数进行固定旋转补偿。

## 运行要求
- Android 8.0+（API 26），建议 Android 13/14 设备。
- 设备具备陀螺仪（无陀螺仪直接退出并提示）。
- 支持 OpenGL ES 2.0。

## 权限与适配
- 声明高采样率权限（Android 13+ 针对 `SENSOR_DELAY_FASTEST` 的要求）：
  - `android.permission.HIGH_SAMPLING_RATE_SENSORS` 已在 `AndroidManifest.xml` 中声明。
- 采样率降级回退：如无权限或设备限制，自动从 `FASTEST` 降级到 `GAME`，避免抛出 `SecurityException`。

## 构建与安装
1. 安装 JDK（推荐 JDK 17），在 IDE/环境中设置 `JAVA_HOME`。
2. 使用 Android Studio 打开项目，选择合适的 SDK/NDK。
3. 命令行构建：
   - Windows：`./gradlew.bat :app:assembleDebug`
   - macOS/Linux：`./gradlew :app:assembleDebug`
4. 安装到真机并运行。

## 使用说明
- 打开应用后，若设备无陀螺仪会弹出提示并退出。
- 触控交互：
  - 单指拖动：环绕相机视角（水平为偏航、垂直为俯仰）。
  - 双指缩放：调整相机远近。
- 视野中：
  - 地面网格与方向轴，原点为当前融合的参考点。
  - 设备位置用轨迹表示（连续点连线）。

## 调参与稳定性
- ZUPT（零速更新）：在静止约 20–40ms 内触发，速度强约束归零，并使用加速度重力矫正倾角。
- 速度阻尼：加速度与角速度近零时指数衰减，极小速度软零处理，抑制停止后的“滑行”。
- 线性加速度低通：`alpha=0.85`，幅值极小时直接置零，减少噪声积分。
- 磁力更新：仅在干扰较低与角速度较小时更新，权重降低，避免航向突变。
- 气压高度：噪声增大，降低短时高度抖动对 Z 的影响。

如需进一步稳定：
- 增大磁力噪声或收紧更新阈值。
- 提高线性加速度滤波 `alpha` 或增大置零幅值阈。
- 提升 ZUPT 强度（更低的速度噪声）或提高速度阻尼系数。

## 常见问题
- Android 13+ 启动崩溃（`SecurityException`）
  - 原因：未声明高采样率权限却以 `FASTEST` 注册传感器。
  - 解决：已在清单添加权限，且代码中自动降级采样率并 `Toast` 提示。
- 构建报错：Gradle 找不到 JDK
  - 解决：安装 JDK 并设置 `JAVA_HOME`，在 IDE/Gradle 指定 JDK。

## 目录结构（简要）
- `app/src/main/java/com/olsc/t6dof/`：`MainActivity`、渲染与传感器管理。
- `app/src/main/cpp/`：`fusion.cpp`（JNI + 状态）、`ekf.cpp/.h`（滤波器）。
- `app/src/main/AndroidManifest.xml`：权限与入口 Activity。

## 许可
- 个人/学习用途示例。若需商用，请自行评估并完善健壮性与安全性。
