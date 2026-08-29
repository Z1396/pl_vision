# stage9_fast_tf —— fast_tf 模块（强类型坐标变换树）

## 功能分析

fast_tf（真实项目约 2,713 行）的核心思想：**每个坐标系是一个独立的 C++ 类型**，
父子链在编译期由类型层级表达（`frame.hpp` 的 FAST_TF_FRAME 宏）。
把 odom 系向量传给 camera 系函数不再是运行期 bug，而是编译错误。

| 文件 | 内容 |
|---|---|
| `frame.hpp` | 帧类型层级（`parent_frame` 编译期父子链）+ 运行期名字注册表 |
| `transform.hpp` | `EdgeTransform<Child>`（child→parent 边变换）、`Vector3<F>`（F 系下的点）、`operator*` 的 requires 约束、`from_rpy`/`from_translation`（boot.cpp 同款构造） |
| `buffer.hpp` | 每帧一个 `TfBuffer<F>`（时间戳保序 ring），`lookup(t)`：平移线性插值 + 旋转 slerp；早于最老样本报错，晚于最新 clamp |
| `validation.hpp` | 注册表环检测（child→parent 链上行必须终于根） |

## 运行方法

```bash
sudo apt install libeigen3-dev   # 依赖
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/demo
```

## 预期输出（节选）

```
=== stage9_fast_tf ===
[test 1] 链式变换（手算对照）
  typed: (2.855715115, 1.296037928, 3.026651604)
  hand:  (2.855715115, 1.296037928, 3.026651604)
  误差 = 0.00e+00
[test 2] 时间戳插值
  插值 yaw = 0.200000000, 期望 0.2
[test 3] 跨 ±pi 旋转插值
  mid yaw = 3.141592654（等效 ±pi = 3.141592654）
[test 4] 环检测
  错误信息: cycle detected: fake_a -> fake_b -> fake_a

ALL TESTS PASSED
```

**编译期错误演示**（demo 中以注释存在，放开即报错）：

```cpp
const auto bad1 = t_pitch * p_cam;          // 点不是 gimbal_pitch_t 系
EdgeTransform<gimbal_yaw_t> bad2 = t_cam;   // 不同边类型不可赋值
const auto bad3 = t_yaw * t_yaw;            // parent(B) != A，链拼不上
```

## 与真实项目的对应

- 帧链 `world → odom → gimbal_yaw → gimbal_pitch → camera` 复刻
  `boot.cpp` 的 `init_coordinate_system`（`from_rpy`/`from_translation` 逐边构造）。
- demo 直接 include 本 stage 的 `include/fast_tf/`（与 `crates/fast_tf/src` 同构），
  真实项目把 include 路径指向后者即可。

## 为什么其余 4 个模块不建独立 stage

| 模块 | 不建原因 | 替代学习方案 |
|---|---|---|
| `quanta` | 编码后端绑定特定硬件（ax_encode_backend 约 2,507 行），依赖 FFmpeg/厂商编解码，无硬件跑不出结果 | 读 `foxglove_systems.cpp` 的传输模式分支，理解 MCP/WebSocket 双路 |
| `hardware`（hik_camera/at_gimbal） | 需要真实相机/串口硬件 + 厂商 SDK | thread_affinity 已由 stage6 覆盖；串口协议看 `talos_gimbal/packet.hpp` 的 static_assert 设计 |
| `hardware_daedalus` | 共享内存对端是 Daedalus 仿真进程，单独跑无意义 | 进阶：用 POSIX shm 自己写对端喂帧 |
| `fcs_visualization` | 依赖 Foxglove 服务 + 全流水线数据 | 已在 l1/l2 可视化系统上做过完整注释分析 |
