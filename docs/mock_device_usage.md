# 模拟设备使用说明

## 概述

为了解决在没有实际相机设备和USB转CAN设备的情况下进行测试的问题，我们实现了虚拟设备模拟模块。这些模块可以模拟真实的相机和CAN板设备，让程序能够在无硬件连接的情况下正常编译和运行。

## 新增文件

### 1. 虚拟相机模块
- **头文件**: `io/mock_camera.hpp`
- **实现文件**: `io/mock_camera.cpp`
- **功能**: 
  - 生成模拟的相机图像（1280x720分辨率）
  - 在图像中添加移动的装甲板目标
  - 添加噪声以模拟真实相机环境
  - 实时显示帧数和目标位置信息

### 2. 虚拟CAN板模块
- **头文件**: `io/mock_cboard.hpp`
- **实现文件**: `io/mock_cboard.cpp`
- **功能**:
  - 生成模拟的IMU姿态数据（四元数）
  - 模拟云台的运动（yaw、pitch、roll）
  - 提供弹速数据（默认27m/s）
  - 支持时间戳查询和插值

### 3. 修改的文件
- **tests/auto_aim_test.cpp**: 添加模拟模式支持
- **io/CMakeLists.txt**: 添加模拟模块到编译系统

## 使用方法

### 启用模拟模式

使用 `-m` 或 `--mock-mode` 参数启用模拟模式：

```bash
cd /home/pldx/Desktop/pl_vision/build
./auto_aim_test -m -g
```

参数说明：
- `-m` 或 `--mock-mode`: 启用模拟设备模式
- `-g` 或 `--enable-gui`: 启用GUI显示
- `-c` 或 `--config-path`: 指定配置文件路径（默认：configs/standard3.yaml）

### 完整示例

```bash
# 模拟模式 + GUI显示
./auto_aim_test --mock-mode --enable-gui

# 模拟模式 + 指定配置文件
./auto_aim_test -m -c configs/standard3.yaml

# 离线视频模式（原有功能）
./auto_aim_test assets/demo/3m -g
```

## 模拟设备特性

### MockCamera

1. **图像生成**:
   - 背景色：深蓝色（RGB: 20, 20, 30）
   - 添加高斯噪声
   - 中心十字标记
   - 四角矩形标记

2. **移动目标**:
   - 青色装甲板（RGB: 0, 200, 255）
   - 白色中心点
   - 目标在图像中自动移动
   - 碰到边界时反弹

3. **信息显示**:
   - 当前帧数
   - 目标位置坐标

### MockCBoard

1. **IMU数据**:
   - 生成10分钟的IMU数据
   - 模拟云台的正弦运动
   - 支持时间戳插值查询

2. **弹速**:
   - 默认：27 m/s
   - 可通过配置文件修改

3. **工作模式**:
   - 默认：auto_aim
   - 支持idle、small_buff、big_buff、outpost

## 与原模式的对比

| 特性 | 模拟模式 | 离线视频模式 |
|------|---------|-------------|
| 图像来源 | 程序生成 | 视频文件 |
| IMU数据 | 程序生成 | TXT文件 |
| 硬件需求 | 无 | 无 |
| 实时性 | 实时生成 | 按文件播放 |
| 目标控制 | 程序控制 | 固定视频 |
| 适用场景 | 算法测试 | 离线验证 |

## 测试验证

### 1. 编译验证

```bash
cd /home/pldx/Desktop/pl_vision/build
make auto_aim_test
```

### 2. 运行验证

```bash
# 测试模拟模式（无GUI）
./auto_aim_test -m

# 测试模拟模式（有GUI）
./auto_aim_test -m -g
```

### 3. 功能验证

程序启动后，您应该看到：
- ✅ 日志显示 "=== 启用模拟设备模式 ==="
- ✅ 日志显示 "MockCamera initialized: 1280x720"
- ✅ 日志显示 "MockCBoard initialized"
- ✅ 实时显示FPS和处理时间
- ✅ GUI显示模拟图像（如果启用-g参数）

## 技术实现

### MockCamera核心算法

1. **图像生成**:
   ```cpp
   img.create(height_, width_, CV_8UC3);
   img.setTo(cv::Scalar(20, 20, 30));  // 背景色
   ```

2. **目标运动**:
   ```cpp
   target_x_ += target_vx_;
   target_y_ += target_vy_;
   // 边界检测和反弹
   ```

3. **噪声添加**:
   ```cpp
   cv::Mat noise(img.size(), img.type());
   cv::randn(noise, 0, 5);
   cv::add(img, noise, img);
   ```

### MockCBoard核心算法

1. **IMU数据生成**:
   ```cpp
   yaw_rate_ = 0.5 * sin(time * 0.5);
   pitch_rate_ = 0.3 * cos(time * 0.3);
   roll_rate_ = 0.2 * sin(time * 0.7);
   ```

2. **四元数插值**:
   ```cpp
   Eigen::Quaterniond q = prev_q.slerp(alpha, next_q);
   ```

## 注意事项

1. **性能**: 模拟模式会生成大量IMU数据，首次启动可能需要几秒钟
2. **内存**: MockCBoard会预生成10分钟的IMU数据，占用约100MB内存
3. **兼容性**: 模拟模式与离线视频模式互斥，不能同时使用
4. **调试**: 使用`-g`参数可以实时查看模拟图像效果

## 扩展建议

1. **自定义目标**: 可以修改`MockCamera::addMovingTarget()`来生成不同的目标模式
2. **IMU轨迹**: 可以修改`MockCBoard::generateMockIMUData()`来模拟不同的运动轨迹
3. **噪声控制**: 可以调整噪声参数来模拟不同的环境条件
4. **多目标**: 可以扩展为支持多个移动目标

## 故障排除

### 问题1: 编译错误
```
error: unknown type name 'Mode'
```
**解决**: 确保在`mock_cboard.hpp`中包含了`io/cboard.hpp`

### 问题2: 运行时崩溃
```
Segmentation fault
```
**解决**: 检查配置文件路径是否正确，确保`configs/standard3.yaml`存在

### 问题3: 图像显示异常
```
图像全黑或全白
```
**解决**: 检查MockCamera的初始化参数，确保width和height设置正确

## 总结

通过实现虚拟设备模拟模块，我们成功解决了硬件缺失导致的测试环境限制问题。现在您可以在没有实际设备的情况下：
- ✅ 正常编译和运行程序
- ✅ 测试自瞄算法的核心逻辑
- ✅ 验证数据处理流程
- ✅ 调试和优化算法性能

这为算法开发和测试提供了极大的便利！