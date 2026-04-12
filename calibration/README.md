# 视觉标定系统操作手册

## 目录

1. [程序概述](#1-程序概述)
2. [环境配置要求](#2-环境配置要求)
3. [程序功能说明](#3-程序功能说明)
4. [详细操作步骤](#4-详细操作步骤)
5. [常见问题及解决方案](#5-常见问题及解决方案)
6. [注意事项与最佳实践](#6-注意事项与最佳实践)
7. [坐标系说明](#7-坐标系说明)

---

## 1. 程序概述

### 1.1 标定系统简介

本标定系统是RM2025视觉自动瞄准系统的重要组成部分，用于标定相机内外参和手眼关系，确保目标定位的准确性。

### 1.2 程序组成

```
calibration/
├── capture.cpp                      # 图像与IMU数据采集程序
├── calibrate_camera.cpp            # 相机内外参标定程序
├── calibrate_handeye.cpp           # 圆点标定板相机标定程序
├── calibrate_robotworld_handeye.cpp # 机器人世界坐标系手眼标定程序
└── split_video.cpp                 # 视频分帧工具
```

### 1.3 标定流程

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   1. 数据采集   │ -> │  2. 相机标定   │ -> │  3. 手眼标定   │
│  capture.cpp   │    │ calibrate_     │    │ calibrate_      │
│                 │    │  camera.cpp    │    │ robotworld_     │
│                 │    │                │    │ handeye.cpp     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                                                │
        ▼                                                ▼
  采集图像和IMU                            输出R_camera2gimbal和
  四元数数据                              t_camera2gimbal
```

---

## 2. 环境配置要求

### 2.1 硬件要求

| 设备 | 要求 |
|------|------|
| 相机 | 海康威视/大华工业相机 |
| 标定板 | 15×15 圆点标定板（推荐）或棋盘格标定板 |
| CAN总线 | 能读取IMU四元数的控制板 |
| 计算机 | Ubuntu 20.04+ |

### 2.2 软件依赖

```bash
# 必需的系统库
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libopencv-dev \
    libeigen3-dev \
    libyaml-cpp-dev

# 项目依赖（已在项目中包含）
# - fmt
# - Eigen3
# - OpenCV
# - yaml-cpp
```

### 2.3 配置文件准备

在运行程序前，需要准备 `configs/calibration.yaml` 配置文件：

```yaml
# 标定板参数
pattern_cols: 15          # 圆点列数
pattern_rows: 15          # 圆点行数
chessboard_square_size_mm: 11.00  # 方格尺寸（棋盘格用）
center_distance_mm: 40    # 圆点间距（圆点板用）

# 云台→IMU旋转矩阵（单位矩阵表示对齐）
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 相机参数
camera_name: "hikrobot"
gamma: 0.5
exposure_ms: 10
gain: 10.0
vid_pid: "174F:2435"

# CAN总线参数
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
can_interface: "can0"
```

---

## 3. 程序功能说明

### 3.1 capture.cpp - 数据采集程序

**功能**：采集标定所需的图像和IMU四元数数据

**输入**：
- 配置文件（YAML格式）

**输出**：
- `{序号}.jpg` - 相机图像
- `{序号}.txt` - 对应的四元数（格式：w x y z）

**使用方法**：
```bash
./capture [-c <配置文件>] [-o <输出文件夹>]
# 示例
./capture -c configs/calibration.yaml -o assets/img_with_q
```

### 3.2 calibrate_camera.cpp - 相机内外参标定

**功能**：标定相机的内参矩阵和畸变系数

**输入**：
- 包含棋盘格图像的文件夹

**输出**：
- 相机内参矩阵 `camera_matrix`
- 畸变系数 `distort_coeffs`
- 重投影误差

**使用方法**：
```bash
./calibrate_camera <输入文件夹> [-c <配置文件>]
# 示例
./calibrate_camera assets/img_with_q -c configs/calibration.yaml
```

### 3.3 calibrate_handeye.cpp - 圆点标定板相机标定

**功能**：使用圆点标定板进行相机标定

**使用方法**：
```bash
./calibrate_handeye <输入文件夹> [-c <配置文件>]
# 示例
./calibrate_handeye assets/img_with_q -c configs/calibration.yaml
```

### 3.4 calibrate_robotworld_handeye.cpp - 手眼标定（核心程序）

**功能**：
1. 实现"眼在手上"(Eye-in-Hand)模型的手眼标定
2. 计算相机相对于云台的安装位置和角度
3. 同时计算标定板在世界坐标系中的位置

**输入**：
- 包含图像和四元数的文件夹（由capture.cpp生成）

**输出**：
- `R_camera2gimbal` - 相机→云台旋转矩阵
- `t_camera2gimbal` - 相机→云台平移向量
- 相机相对于理想安装姿态的偏角

**使用方法**：
```bash
./calibrate_robotworld_handeye <输入文件夹> [-c <配置文件>]
# 示例
./calibrate_robotworld_handeye assets/img_with_q -c configs/calibration.yaml
```

### 3.5 split_video.cpp - 视频分帧工具

**功能**：将视频文件按指定帧范围分割

**使用方法**：
```bash
./split_video <输入路径> [-s <起始帧>] [-e <结束帧>] [-p <输出路径>]
# 示例
./split_video records/Big/2024-05-14_11-6-26 -s 0 -e 1000 -p output/Big_clip
```

---

## 4. 详细操作步骤

### 4.1 步骤一：准备工作

#### 4.1.1 硬件连接

```
┌─────────────┐      CAN总线       ┌─────────────┐
│   计算机    │ ◄──────────────► │   控制板    │
└──────┬──────┘                  └──────┬──────┘
       │                                   │
       │ 网线                              │ CAN
       ▼                                   ▼
┌─────────────┐                      ┌─────────────┐
│   工业相机  │                      │   云台+IMU  │
└─────────────┘                      └─────────────┘
```

#### 4.1.2 标定板准备

1. 打印15×15圆点标定板
2. 粘贴在平整的平面上
3. 记录圆点间距（通常40mm）

#### 4.1.3 确认CAN总线连接

```bash
# 检查CAN接口
ip link show can0

# 如果没有，启动CAN接口
sudo ip link set can0 up type can bitrate 1000000
```

### 4.2 步骤二：数据采集

1. **编译程序**
   ```bash
   cd build
   cmake ..
   make -j$(nproc)
   ```

2. **运行采集程序**
   ```bash
   cd build/calibration
   ./capture -c ../../configs/calibration.yaml -o ../../assets/img_with_q
   ```

3. **采集数据**
   - 程序启动后会显示实时画面
   - 移动云台拍摄标定板的不同姿态
   - 按 **S键** 保存当前帧
   - 按 **Q键** 退出程序

4. **采集要求**
   - 至少采集 **15-20组** 不同姿态的数据
   - 覆盖云台的各个角度范围
   - 标定板应在画面中清晰可见
   - 每次保存时确保IMU数据稳定

### 4.3 步骤三：相机内外参标定（可选）

1. **运行相机标定**
   ```bash
   ./calibrate_camera ../../assets/img_with_q -c ../../configs/calibration.yaml
   ```

2. **查看结果**
   - 程序会显示重投影误差
   - 误差 < 0.5px 为优秀
   - 误差 < 1.0px 为良好

### 4.4 步骤四：手眼标定（核心步骤）

1. **运行手眼标定**
   ```bash
   ./calibrate_robotworld_handeye ../../assets/img_with_q -c ../../configs/calibration.yaml
   ```

2. **查看输出结果**
   ```
   ==================== 标定结果 ====================
   
   相机→云台旋转矩阵 R_camera2gimbal:
     [0.898, -0.395, -0.193]
     [0.393,  0.918, -0.046]
     [0.195, -0.035,  0.980]
   
   相机→云台平移向量 t_camera2gimbal (m):
     [0.1256, -0.0596, 0.0639]
   
   相机相对于理想安装姿态的偏角:
     yaw   = -153.60°
     pitch = -63.94°
     roll  = -116.06°
   ```

3. **更新配置文件**
   将标定结果更新到 `configs/standard3.yaml`：
   
   ```yaml
   # 替换原有的R_camera2gimbal和t_camera2gimbal
   R_camera2gimbal: [0.898, -0.395, -0.193, 0.393, 0.918, -0.046, 0.195, -0.035, 0.980]
   t_camera2gimbal: [0.1256, -0.0596, 0.0639]
   ```

---

## 5. 常见问题及解决方案

### 5.1 图像采集问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 无法打开相机 | 相机未连接/驱动未安装 | 检查相机连接，安装对应驱动 |
| 图像模糊 | 曝光/对焦不当 | 调整exposure_ms参数 |
| 标定板检测失败 | 标定板不清晰/光照不足 | 改善光照，确保标定板平整 |

### 5.2 标定精度问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 重投影误差过大 | 样本数不足/标定板不准确 | 增加样本数量，检查标定板尺寸 |
| 手眼标定失败 | 数据不足/姿态变化太小 | 确保采集足够的不同姿态数据 |
| 相机偏角异常 | R_gimbal2imubody设置错误 | 检查配置文件中的矩阵值 |

### 5.3 通信问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| CAN总线无响应 | CAN接口未启动 | 执行 sudo ip link set can0 up |
| IMU数据为0 | 四元数CAN ID错误 | 检查配置文件中的quaternion_canid |

---

## 6. 注意事项与最佳实践

### 6.1 数据采集注意事项

1. **标定板要求**
   - 使用标准的15×15圆点标定板
   - 确保标定板表面平整无褶皱
   - 圆点间距需精确测量并记录

2. **采集姿态要求**
   - 覆盖云台的所有运动范围
   - 包括正对、侧视、俯视等多个角度
   - 标定板占画面1/3-1/2大小为宜

3. **环境要求**
   - 光照均匀，避免强光直射
   - 背景简洁，减少干扰
   - 标定板与背景对比度明显

### 6.2 标定过程注意事项

1. **参数验证**
   - 重投影误差 < 1.0像素
   - 标定样本数 ≥ 15组
   - 手眼标定结果合理

2. **坐标系检查**
   - 确认R_gimbal2imubody为单位矩阵（默认对齐）
   - 验证相机偏角在合理范围内

3. **结果验证**
   - 静态目标的世界坐标应保持稳定
   - 云台运动时观测值应合理变化

### 6.3 最佳实践建议

1. **定期标定**
   - 比赛前进行一次完整标定
   - 更换相机镜头后重新标定
   - 长时间不使用后重新标定

2. **数据保存**
   - 保留每次标定的原始数据
   - 记录标定日期和环境条件
   - 建立标定历史记录

3. **交叉验证**
   - 使用不同数据多次标定对比结果
   - 在实际场景中验证标定效果
   - 定期检查标定参数是否有变化

---

## 7. 坐标系说明

### 7.1 坐标系定义

| 坐标系 | 原点 | 轴向定义 |
|--------|------|----------|
| **世界坐标系** | 云台旋转中心在地面投影 | Z轴垂直向上，X轴向前，Y轴向右 |
| **云台坐标系** | 云台旋转中心 | 随云台运动，Z轴垂直向上 |
| **相机坐标系** | 相机光心 | Z轴沿光轴，X/Y轴与图像对应 |
| **图像坐标系** | 图像左上角 | U向右，V向下 |

### 7.2 坐标转换链

```
图像坐标 → 相机坐标 → 云台坐标 → 世界坐标
   ↓           ↓           ↓          ↓
  像素      PnP求解    手眼标定    IMU四元数
```

### 7.3 关键转换公式

**云台→世界（当R_gimbal2imubody为单位矩阵时）：**
```
R_gimbal2world = R_imubody2imuabs
```

**相机→云台：**
```
xyz_gimbal = R_camera2gimbal × xyz_camera + t_camera2gimbal
```

**云台→世界：**
```
xyz_world = R_gimbal2world × xyz_gimbal
```

---

## 附录：快速参考命令

```bash
# 1. 编译项目
cd build && cmake .. && make -j$(nproc)

# 2. 采集数据
./build/calibration/capture -c configs/calibration.yaml -o assets/img_with_q

# 3. 相机标定（可选）
./build/calibration/calibrate_camera assets/img_with_q -c configs/calibration.yaml

# 4. 手眼标定
./build/calibration/calibrate_robotworld_handeye assets/img_with_q -c configs/calibration.yaml

# 5. 视频分帧
./build/calibration/split_video records/Big/2024-05-14_11-6-26 -s 0 -e 1000 -p output
```

---

**文档版本**：v1.0  
**更新时间**：2025年
**适用版本**：RM2025视觉系统
