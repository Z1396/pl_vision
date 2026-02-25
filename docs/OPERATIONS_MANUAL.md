# pl_vision网页调试器 操作手册

## 目录

1. [项目概述](#项目概述)
2. [环境搭建](#环境搭建)
3. [快速开始](#快速开始)
4. [功能模块说明](#功能模块说明)
5. [日常维护](#日常维护)
6. [常见问题处理](#常见问题处理)
7. [故障排查](#故障排查)

---

## 项目概述

pl_vision网页调试器是一个基于计算机视觉的自动瞄准系统，提供实时波形可视化功能，用于调试和分析系统性能。

**项目路径：** `/home/pldx/Desktop/pl_vision`

**主要功能：**
- 实时视频流显示
- 11个调试数据的波形可视化
- WebSocket实时数据传输
- PlotJuggler风格的交互界面

---

## 环境搭建

### 系统要求

- **操作系统：** Linux (Ubuntu 20.04+)
- **编译器：** GCC 9.0+
- **CMake：** 3.10+
- **Python：** 3.6+

### 依赖库

#### 必需依赖
```bash
# OpenCV
sudo apt-get install libopencv-dev

# OpenSSL
sudo apt-get install libssl-dev

# nlohmann/json
sudo apt-get install nlohmann-json3-dev

# CMake
sudo apt-get install cmake build-essential

# Python HTTP服务器
sudo apt-get install python3
```

#### 可选依赖
```bash
# 调试工具
sudo apt-get install gdb valgrind

# 网络工具
sudo apt-get install net-tools lsof
```

### 编译项目

```bash
# 进入项目目录
cd /home/pldx/Desktop/pl_vision

# 创建构建目录
mkdir -p build
cd build

# 配置CMake
cmake ..

# 编译项目
make -j$(nproc)
```

**编译成功后，可执行文件位于：** `build/auto_aim_test`

---

## 快速开始

### 第一次运行

#### 1. 启动HTTP服务器

```bash
# 进入web目录
cd /home/pldx/Desktop/pl_vision/web

# 启动HTTP服务器（端口8000）
python3 -m http.server 8000
```

**说明：** HTTP服务器用于提供静态文件服务，包括HTML、CSS、JavaScript和ECharts库。

#### 2. 启动auto_aim_test程序

打开新的终端窗口：

```bash
# 进入项目目录
cd /home/pldx/Desktop/pl_vision

# 运行auto_aim_test程序
./build/auto_aim_test assets/demo/3m
```

**说明：** 
- `assets/demo/3m` 是视频文件路径（不含扩展名）
- 程序会自动查找 `3m.avi` 和 `3m.txt` 文件
- 程序启动后会监听8080端口，提供WebSocket服务

#### 3. 访问Web界面

在浏览器中打开：
```
http://localhost:8000/index.html
```

**说明：** 
- 如果使用远程访问，将 `localhost` 替换为服务器IP地址
- 例如：`http://192.168.43.247:8000/index.html`

### 验证运行状态

#### 检查HTTP服务器
```bash
lsof -i :8000
```
应该看到python进程监听8000端口。

#### 检查WebSocket服务器
```bash
lsof -i :8080
```
应该看到auto_aim_test进程监听8080端口。

#### 检查程序运行
```bash
ps aux | grep auto_aim_test
```
应该看到auto_aim_test进程在运行。

---

## 功能模块说明

### 1. 视频流模块

**功能：** 实时显示摄像头或视频文件的画面

**显示位置：** 页面顶部视频区域

**数据来源：** 
- 摄像头实时画面
- 视频文件播放（如3m.avi）

**技术实现：**
- 使用OpenCV捕获视频帧
- JPEG编码压缩
- Base64编码传输
- WebSocket实时推送

### 2. 波形可视化模块

**功能：** 实时显示11个调试数据的波形图

**图表列表：**

| 图表名称 | 显示内容 | 数据字段 |
|---------|---------|---------|
| 装甲板位置 | 装甲板X、Y坐标 | armor_x, armor_y |
| 装甲板航向角 | 航向角、原始航向角 | armor_yaw, armor_yaw_raw |
| 云台角度 | 云台Yaw、Pitch角度 | gimbal_yaw, gimbal_pitch |
| 指令角度 | 指令Yaw、Pitch角度 | cmd_yaw, cmd_pitch |
| 目标位置 | 目标X、Y、Z坐标 | x, y, z |
| 目标速度 | 目标VX、VY、VZ速度 | vx, vy, vz |
| 目标角度和旋转速度 | 角度、旋转速度 | a, w |
| 目标尺寸 | 半径、长度、高度 | r, l, h |
| 卡尔曼滤波残差 | Yaw、Pitch、距离、角度残差 | residual_yaw, residual_pitch, residual_distance, residual_angle |
| NIS和NEES | 卡尔曼滤波指标 | nis, nees |
| 帧延迟 | 处理延迟时间 | frame_delay |

**交互功能：**
- **缩放：** 鼠标滚轮或拖拽滑块
- **平移：** 鼠标拖拽图表
- **重置缩放：** 点击"重置缩放"按钮
- **自动滚动：** 点击"自动滚动"按钮切换
- **清除数据：** 点击"清除数据"按钮清空所有图表
- **数据点数量：** 下拉菜单选择100/200/500/1000点

### 3. 信息面板模块

**功能：** 显示关键状态信息

**显示内容：**
- 装甲板数量：当前检测到的装甲板数量
- 帧延迟：当前帧的处理延迟（毫秒）
- 目标ID：当前追踪的目标ID
- 发射状态：是否满足发射条件

### 4. 连接状态模块

**功能：** 显示WebSocket连接状态

**状态指示：**
- **已连接（绿色）：** WebSocket连接正常
- **未连接（红色）：** WebSocket连接断开

---

## 日常维护

### 启动服务

#### 启动HTTP服务器
```bash
cd /home/pldx/Desktop/pl_vision/web
python3 -m http.server 8000
```

#### 启动auto_aim_test
```bash
cd /home/pldx/Desktop/pl_vision
./build/auto_aim_test assets/demo/3m
```

### 停止服务

#### 停止HTTP服务器
在运行HTTP服务器的终端按 `Ctrl + C`

#### 停止auto_aim_test
在运行auto_aim_test的终端按 `Ctrl + C`

或使用命令：
```bash
# 查找进程ID
ps aux | grep auto_aim_test

# 终止进程
kill -9 <PID>
```

### 检查服务状态

#### 检查端口占用
```bash
# 检查HTTP服务器端口（8000）
lsof -i :8000

# 检查WebSocket服务器端口（8080）
lsof -i :8080
```

#### 检查进程运行
```bash
# 检查auto_aim_test进程
ps aux | grep auto_aim_test

# 检查HTTP服务器进程
ps aux | grep "python3 -m http.server"
```

### 日志查看

#### 查看程序输出
auto_aim_test的输出直接显示在终端窗口。

#### 查看浏览器控制台
1. 打开浏览器开发者工具（F12）
2. 切换到"Console"标签
3. 查看日志信息和错误

### 更新代码

#### 重新编译
```bash
cd /home/pldx/Desktop/pl_vision/build
make -j$(nproc)
```

#### 更新前端文件
直接修改 `web/index.html` 文件，刷新浏览器即可看到效果。

### 数据备份

#### 备份视频文件
```bash
cd /home/pldx/Desktop/pl_vision/assets/demo
cp 3m.avi 3m.avi.backup
cp 3m.txt 3m.txt.backup
```

#### 备份配置文件
```bash
cd /home/pldx/Desktop/pl_vision
tar -czf backup_$(date +%Y%m%d).tar.gz web/ assets/demo/
```

---

## 常见问题处理

### 问题1：网页显示"服务不可用"

**症状：** 浏览器访问 `http://localhost:8000/index.html` 显示"服务不可用"

**原因：** HTTP服务器未启动

**解决方案：**
```bash
# 检查HTTP服务器是否运行
lsof -i :8000

# 如果没有运行，启动HTTP服务器
cd /home/pldx/Desktop/pl_vision/web
python3 -m http.server 8000
```

### 问题2：网页显示"未连接"

**症状：** 网页顶部状态显示"未连接"（红色）

**原因：** WebSocket服务器未启动或auto_aim_test程序未运行

**解决方案：**
```bash
# 检查WebSocket服务器是否运行
lsof -i :8080

# 如果没有运行，启动auto_aim_test
cd /home/pldx/Desktop/pl_vision
./build/auto_aim_test assets/demo/3m
```

### 问题3：视频流不显示

**症状：** 视频区域空白或显示加载失败

**原因：** 
1. WebSocket连接失败
2. 视频文件路径错误
3. 程序在mock模式下运行

**解决方案：**
```bash
# 1. 检查WebSocket连接
# 在浏览器控制台查看是否有错误

# 2. 检查视频文件是否存在
ls -l /home/pldx/Desktop/pl_vision/assets/demo/3m.avi
ls -l /home/pldx/Desktop/pl_vision/assets/demo/3m.txt

# 3. 确保程序不在mock模式下运行
./build/auto_aim_test assets/demo/3m
# 不要使用 --mock-mode 参数
```

### 问题4：波形图表不显示

**症状：** 波形图表区域空白

**原因：** 
1. ECharts库未加载
2. 数据未接收到
3. 图表初始化失败

**解决方案：**
```bash
# 1. 检查ECharts文件是否存在
ls -l /home/pldx/Desktop/pl_vision/web/echarts.min.js

# 2. 在浏览器控制台查看错误信息
# 按F12打开开发者工具，查看Console标签

# 3. 强制刷新浏览器
# 按 Ctrl + Shift + R
```

### 问题5：端口被占用

**症状：** 启动服务时提示"Address already in use"

**原因：** 端口被其他程序占用

**解决方案：**
```bash
# 查找占用端口的进程
lsof -i :8000  # HTTP服务器端口
lsof -i :8080  # WebSocket服务器端口

# 终止占用进程
kill -9 <PID>

# 重新启动服务
```

### 问题6：程序崩溃

**症状：** auto_aim_test程序意外退出

**原因：** 
1. 视频文件损坏
2. 内存不足
3. 代码错误

**解决方案：**
```bash
# 1. 检查视频文件
ffprobe /home/pldx/Desktop/pl_vision/assets/demo/3m.avi

# 2. 使用gdb调试
gdb ./build/auto_aim_test
(gdb) run assets/demo/3m
(gdb) bt  # 查看调用栈

# 3. 检查系统资源
free -h  # 查看内存使用
top      # 查看CPU使用
```

### 问题7：数据更新缓慢

**症状：** 波形更新频率低，延迟大

**原因：** 
1. 系统负载高
2. 网络延迟
3. 程序性能问题

**解决方案：**
```bash
# 1. 检查系统负载
top

# 2. 检查网络延迟
ping localhost

# 3. 降低JPEG质量（修改代码）
# 在auto_aim_test.cpp中修改JPEG质量参数
cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 50});
```

### 问题8：浏览器控制台显示WebSocket错误

**症状：** 浏览器控制台显示WebSocket连接错误

**常见错误：**
1. `net::ERR_CONNECTION_REFUSED` - 连接被拒绝
2. `Error during WebSocket handshake` - 握手失败
3. `WebSocket disconnected` - 连接断开

**解决方案：**
```bash
# 1. 检查WebSocket服务器是否运行
lsof -i :8080

# 2. 检查防火墙设置
sudo ufw status

# 3. 检查WebSocket URL是否正确
# 在浏览器中打开index.html，检查WebSocket URL
# 应该是 ws://<IP地址>:8080
```

---

## 故障排查

### 故障排查流程图

```
问题出现
    ↓
检查服务状态
    ↓
├─ HTTP服务器未启动 → 启动HTTP服务器
├─ WebSocket服务器未启动 → 启动auto_aim_test
├─ 端口被占用 → 终止占用进程
└─ 服务正常 → 继续排查
    ↓
检查网络连接
    ↓
├─ 无法访问 → 检查防火墙/网络配置
└─ 连接正常 → 继续排查
    ↓
检查数据传输
    ↓
├─ 无数据 → 检查程序输出/日志
├─ 数据错误 → 检查数据格式/编码
└─ 数据正常 → 继续排查
    ↓
检查前端显示
    ↓
├─ ECharts未加载 → 检查文件路径
├─ 图表初始化失败 → 查看浏览器控制台
└─ 显示正常 → 问题解决
```

### 调试技巧

#### 1. 启用详细日志

在auto_aim_test.cpp中添加日志输出：

```cpp
std::cout << "[DEBUG] WebSocket服务器启动中..." << std::endl;
std::cout << "[DEBUG] 监听端口: 8080" << std::endl;
```

#### 2. 使用浏览器开发者工具

**Console标签：** 查看JavaScript日志和错误

**Network标签：** 查看WebSocket连接和数据传输

**Elements标签：** 检查DOM元素和样式

#### 3. 监控WebSocket消息

在浏览器控制台添加日志：

```javascript
ws.onmessage = function(event) {
    console.log('[WebSocket] 收到消息:', event.data);
    // ... 原有代码
};
```

#### 4. 检查数据格式

在浏览器控制台打印接收到的数据：

```javascript
ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    console.log('[DEBUG] 消息类型:', message.type);
    console.log('[DEBUG] 消息数据:', message.data);
    // ... 原有代码
};
```

### 性能监控

#### 监控系统资源

```bash
# 实时监控CPU和内存使用
top

# 查看内存使用情况
free -h

# 查看磁盘使用情况
df -h
```

#### 监控网络流量

```bash
# 监控网络连接
netstat -an | grep 8080

# 监控WebSocket连接
ss -tlnp | grep 8080
```

#### 监控程序性能

```bash
# 使用time命令测量运行时间
time ./build/auto_aim_test assets/demo/3m

# 使用valgrind检查内存泄漏
valgrind --leak-check=full ./build/auto_aim_test assets/demo/3m
```

---

## 附录

### A. 端口说明

| 端口 | 用途 | 协议 |
|------|------|------|
| 8000 | HTTP服务器 | HTTP |
| 8080 | WebSocket服务器 | WebSocket |

### B. 文件结构

```
pl_vision/
├── assets/
│   └── demo/
│       ├── 3m.avi          # 视频文件
│       └── 3m.txt          # 视频配置文件
├── build/
│   └── auto_aim_test       # 可执行文件
├── docs/
│   ├── DEVELOPMENT_PROCESS.md  # 开发流程文档
│   └── OPERATIONS_MANUAL.md    # 操作手册（本文件）
├── tests/
│   └── auto_aim_test.cpp   # 主程序源代码
├── tools/
│   └── websocket_server.hpp   # WebSocket服务器头文件
└── web/
    ├── index.html          # 主页面
    └── echarts.min.js      # ECharts库
```

### C. 常用命令速查

```bash
# 启动服务
cd /home/pldx/Desktop/pl_vision/web && python3 -m http.server 8000
cd /home/pldx/Desktop/pl_vision && ./build/auto_aim_test assets/demo/3m

# 检查端口
lsof -i :8000
lsof -i :8080

# 查看进程
ps aux | grep auto_aim_test
ps aux | grep "python3 -m http.server"

# 终止进程
kill -9 <PID>

# 编译项目
cd /home/pldx/Desktop/pl_vision/build
cmake ..
make -j$(nproc)
```

### D. 联系方式

如有问题或建议，请联系开发团队。

---

**文档版本：** 1.0  
**最后更新：** 2026-02-25  
**维护者：** z1396
