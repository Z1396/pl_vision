# stage10_math_so2 —— math 模块（SO(2) 李群 + 欧拉角）

## 功能分析

`math` 模块本体只有两个头文件（真实项目约 207 行，header-only）：

| 文件 | 内容 |
|---|---|
| `so2.hpp` | SO(2) 旋转李群：角度构造时用 `std::remainder` 归一化到 `[-pi, pi]`；`operator-` = 流形增量 `Δθ = a - b`；`operator+` = 叠加增量；`inverse()` / `rotation_matrix()` |
| `euler.hpp` | 欧拉角 RPY 与 3x3 旋转矩阵互转（ZYX 内旋：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`），含万向锁特解 |

**为什么不用裸 double 存角度**：`350° - 10°` 裸减法得 `+340°`，而流形上的真实增量是 `-20°`。
EKF 的观测残差、跟踪器的 yaw 解算一旦跨 ±π 边界就会跳变整整一个周期 —— SO2 用类型系统把这层语义固定下来：
凡是经过 SO2 的角差，天然落在 `[-pi, pi]`。

## 运行方法

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/demo   # 退出码 0 = 全部断言通过
```

## 预期输出

```
=== stage10_math_so2 ===
[test 1] SO2 跨 ±pi 边界减法
  SO2:  -0.349 rad (-20.0 deg)
  raw:  340.0 deg
  [PASS] delta == -20 deg (而非 +340 deg)
  [PASS] 结果落在 [-pi, pi]
[test 2] 增量叠加跨界回绕
  (pi - 0.1) + 0.2 = -3.041593, 期望 -(pi - 0.1) = -3.041593
  [PASS] pi+0.1 回绕为 -(pi-0.1)
  [PASS] inverse = -angle
[test 3] euler 与 SO2 混用
  rpy 往返最大误差 = 0.00e+00
  [PASS] R(0,0,yaw) == R(SO2(yaw))
  [PASS] to_rpy(from_rpy(x)) == x
[test 4] 裸 double 反例（观测残差视角）
  raw residual:  +340.0 deg   <- EKF 会把 0.4 rad 的残差放大成 6 rad
  SO2 residual:   -20.0 deg   <- 真实的最短角差
  [PASS] raw 与 SO2 相差整整一个周期

ALL TESTS PASSED
```

## 与真实项目的对应

- demo 直接 include 本 stage 的 `include/math/`（与 `crates/math/src` 同构）。
  真实项目里把 CMake 的 include 路径指向 `crates/math/src` 即可无缝替换。
- 跟踪器 yaw 状态量、装甲板角度观测，都应经由 SO2 做"减"（残差）与"加"（状态更新）。
