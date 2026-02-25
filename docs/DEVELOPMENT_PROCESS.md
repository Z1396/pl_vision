# 网页调试项目开发流程文档

## 项目概述

本项目为pl_vision网页调试器计算机视觉系统，实现了基于Web的实时波形可视化功能，用于显示自动瞄准系统的调试数据。

**项目路径：** `/home/pldx/Desktop/pl_vision`

**核心功能：**
- 实时视频流传输
- WebSocket数据通信
- 11个波形图表的实时可视化
- PlotJuggler风格的交互界面

---

## 开发时间线

### 第一阶段：问题诊断（初始阶段）

**问题描述：**
用户反馈网页显示"服务不可用"，摄像头画面和数据调试画面无法显示。

**诊断过程：**
1. 检查WebSocket连接状态
2. 验证HTTP服务器运行状态
3. 检查auto_aim_test程序是否运行

**发现的问题：**
1. WebSocket服务器未启动
2. HTTP服务器未运行
3. auto_aim_test程序未执行

**解决方案：**
1. 启动HTTP服务器：`python3 -m http.server 8000`
2. 运行auto_aim_test程序：`./build/auto_aim_test assets/demo/3m`
3. 验证服务端口：`lsof -i :8080` 和 `lsof -i :8000`

---

### 第二阶段：WebSocket连接问题

**问题现象：**
浏览器控制台显示 `net::ERR_CONNECTION_REFUSED`

**问题分析：**
1. WebSocket服务器未正确初始化
2. 端口8080被占用
3. 程序崩溃导致服务停止

**解决方案：**
1. 检查端口占用：`lsof -i :8080`
2. 终止占用进程：`kill -9 <PID>`
3. 重新启动auto_aim_test程序
4. 添加程序监控，确保持续运行

**技术细节：**
- WebSocket服务器在auto_aim_test.cpp中初始化
- 使用8080端口进行WebSocket通信
- 需要确保程序在后台持续运行

---

### 第三阶段：JSON序列化错误

**问题现象：**
浏览器控制台显示JSON解析错误，数据无法正确显示

**问题分析：**
1. JPEG图像数据直接序列化为JSON导致UTF-8编码错误
2. 二进制数据无法直接转换为JSON字符串

**解决方案：**
在WebSocketServer::broadcastVideo函数中添加Base64编码：

```cpp
// 修改前
json message;
message["type"] = "video";
message["data"] = jpeg_data;  // 直接使用二进制数据

// 修改后
json message;
message["type"] = "video";
std::string base64_data = base64_encode(jpeg_data);
message["data"] = base64_data;  // 使用Base64编码
```

**实现细节：**
- 添加Base64编码函数
- 在发送前对JPEG数据进行编码
- 在前端解码：`img.src = 'data:image/jpeg;base64,' + message.data`

---

### 第四阶段：ECharts加载失败

**问题现象：**
波形图表无法显示，浏览器控制台显示ECharts加载失败

**问题分析：**
1. CDN访问被阻止
2. 网络连接问题
3. Tracking Prevention阻止了CDN资源

**解决方案：**
1. 下载ECharts本地版本：
   ```bash
   curl -L -o echarts.min.js https://cdn.jsdelivr.net/npm/echarts@5.4.3/dist/echarts.min.js
   ```

2. 修改HTML文件，引用本地文件：
   ```html
   <!-- 修改前 -->
   <script src="https://cdn.jsdelivr.net/npm/echarts@5.4.3/dist/echarts.min.js"></script>
   
   <!-- 修改后 -->
   <script src="echarts.min.js"></script>
   ```

**技术细节：**
- 将echarts.min.js放在web目录下
- 确保HTTP服务器正确提供静态文件
- 避免依赖外部CDN，提高稳定性

---

### 第五阶段：WebSocket握手失败

**问题现象：**
浏览器控制台显示：`Error during WebSocket handshake: Incorrect 'Sec-WebSocket-Accept' header value`

**问题分析：**
1. WebSocket握手响应头计算错误
2. 自定义SHA1实现有bug
3. Sec-WebSocket-Accept值不正确

**解决方案：**
1. 检查WebSocket握手实现
2. 发现自定义SHA1函数存在问题
3. 替换为OpenSSL的SHA1函数：

```cpp
// 修改前
void WebSocketServer::sha1(const unsigned char* data, size_t len, unsigned char* hash) {
    // 自定义SHA1实现（有bug）
}

// 修改后
#include <openssl/sha.h>

void WebSocketServer::sha1(const unsigned char* data, size_t len, unsigned char* hash) {
    SHA1(data, len, hash);  // 使用OpenSSL的SHA1
}
```

**实现细节：**
- 添加OpenSSL库依赖
- 在CMakeLists.txt中链接libssl
- 确保SHA1计算正确

---

### 第六阶段：连接稳定性问题

**问题现象：**
WebSocket连接建立后立即断开，错误代码1006

**问题分析：**
1. 客户端断开后服务器未正确处理
2. 连接状态监控缺失
3. getsockopt检测连接状态

**解决方案：**
1. 添加连接监控线程
2. 使用getsockopt检测连接状态
3. 实现自动重连机制

```cpp
// 添加连接监控
void WebSocketServer::monitorConnections() {
    while (running_) {
        std::vector<int> disconnected;
        for (int client_fd : clients_) {
            int error = 0;
            socklen_t len = sizeof(error);
            getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &error, &len);
            if (error != 0) {
                disconnected.push_back(client_fd);
            }
        }
        
        for (int fd : disconnected) {
            clients_.erase(fd);
            close(fd);
        }
        
        usleep(100000);  // 100ms
    }
}
```

**技术细节：**
- 使用SO_ERROR选项检测连接状态
- 定期检查所有客户端连接
- 自动清理断开的连接

---

### 第七阶段：SIGPIPE导致程序崩溃

**问题现象：**
程序在客户端断开后崩溃

**问题分析：**
1. 向已关闭的socket发送数据触发SIGPIPE
2. 默认行为是终止程序
3. 未处理SIGPIPE信号

**解决方案：**
在WebSocketServer::start()中忽略SIGPIPE信号：

```cpp
signal(SIGPIPE, SIG_IGN);
```

**技术细节：**
- 忽略SIGPIPE信号
- 改为检查send返回值
- 处理EPIPE错误

---

### 第八阶段：视频传输过大

**问题现象：**
浏览器显示"Could not decode a text frame as UTF-8"

**问题分析：**
1. JPEG质量过高导致数据量过大
2. 单帧数据超过WebSocket传输限制
3. 数据分片传输不完整

**解决方案：**
1. 降低JPEG质量参数
2. 确保完整数据传输
3. 添加发送循环

```cpp
// 降低JPEG质量
cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 60});

// 确保完整传输
size_t total_sent = 0;
while (total_sent < frame.length()) {
    ssize_t sent = send(client_fd, frame.c_str() + total_sent, 
                      frame.length() - total_sent, 0);
    if (sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            usleep(1000);
            continue;
        }
        break;
    }
    total_sent += sent;
}
```

**技术细节：**
- JPEG质量从95降至60
- 数据大小从637KB降至约89KB
- 使用循环确保完整传输

---

### 第九阶段：数据字段缺失

**问题现象：**
当没有检测到目标时，波形图表不显示数据

**问题分析：**
1. 只在有目标时发送数据
2. 缺少默认值处理
3. 数据字段不完整

**解决方案：**
在auto_aim_test.cpp中添加else分支，设置默认值：

```cpp
// 装甲板数据默认值
if (!armors.empty()) {
    const auto & armor = armors.front();
    data["armor_x"] = armor.xyz_in_world[0];
    data["armor_y"] = armor.xyz_in_world[1];
    data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    data["armor_center_x"] = armor.center_norm.x;
    data["armor_center_y"] = armor.center_norm.y;
} else {
    // 没有检测到装甲板时，发送默认值
    data["armor_x"] = 0.0;
    data["armor_y"] = 0.0;
    data["armor_yaw"] = 0.0;
    data["armor_yaw_raw"] = 0.0;
    data["armor_center_x"] = 0.0;
    data["armor_center_y"] = 0.0;
}

// 目标数据默认值
if (!targets.empty()) {
    // ... 已有代码 ...
} else {
    // 没有检测到目标时，发送默认值
    data["x"] = 0.0;
    data["vx"] = 0.0;
    data["y"] = 0.0;
    data["vy"] = 0.0;
    data["z"] = 0.0;
    data["vz"] = 0.0;
    data["a"] = 0.0;
    data["w"] = 0.0;
    data["r"] = 0.0;
    data["l"] = 0.0;
    data["h"] = 0.0;
    data["last_id"] = -1;
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0;
    data["nees_fail"] = 0;
    data["recent_nis_failures"] = 0;
}
```

**技术细节：**
- 为所有数据字段添加默认值
- 确保数据完整性
- 使用0值表示无检测状态

---

### 第十阶段：视频源错误

**问题现象：**
显示的视频与用户提供的视频文件不一致

**问题分析：**
1. 程序在mock模式下运行
2. 使用模拟数据而非实际视频
3. 未指定正确的视频路径

**解决方案：**
运行程序时指定视频路径：

```bash
# 修改前
./build/auto_aim_test --mock-mode

# 修改后
./build/auto_aim_test assets/demo/3m
```

**技术细节：**
- 移除--mock-mode参数
- 指定实际视频文件路径
- 使用3m.avi作为输入

---

### 第十一阶段：波形可视化实现

**需求：**
实现类似PlotJuggler的波形可视化功能

**实现过程：**

1. **创建基础HTML页面**
   - 引入ECharts库
   - 创建11个图表容器
   - 添加工具栏和信息面板

2. **初始化ECharts图表**
   ```javascript
   function initCharts() {
       armorPositionChart = echarts.init(document.getElementById('armorPositionChart'));
       // ... 初始化其他10个图表
   }
   ```

3. **配置图表选项**
   - 设置公共选项（commonOption）
   - 配置坐标轴、图例、工具提示
   - 添加缩放功能（dataZoom）

4. **实现数据更新**
   ```javascript
   function updateCharts(data) {
       // 更新时间轴
       timeData.push(timeStr);
       
       // 更新各数据系列
       if (data.armor_x !== undefined) armorXData.push(data.armor_x);
       // ... 更新其他数据
       
       // 更新图表
       armorPositionChart.setOption({
           xAxis: { data: timeData },
           series: [{ data: armorXData }, { data: armorYData }]
       });
   }
   ```

5. **添加交互功能**
   - 重置缩放
   - 自动滚动
   - 清除数据
   - 数据点数量选择

**技术细节：**
- 使用window.addEventListener('load', ...)确保页面完全加载
- 单独更新每个图表而非使用forEach
- 添加图表存在性检查
- 实现平滑曲线（smooth: true）

---

### 第十二阶段：波形显示问题

**问题现象：**
部分页面能显示波形，部分页面不能

**问题分析：**
1. 图表初始化时机不当
2. 代码缩进错误导致逻辑问题
3. forEach循环更新图表不正确

**解决方案：**
1. 使用window.addEventListener('load', ...)确保页面完全加载
2. 修复代码缩进，确保所有图表配置在try块内
3. 单独更新每个图表：

```javascript
// 修改前
charts.forEach(chart => {
    chart.setOption({
        xAxis: { data: timeData }
    });
});

// 修改后
if (armorPositionChart) {
    armorPositionChart.setOption({
        xAxis: { data: timeData },
        series: [{ data: armorXData }, { data: armorYData }]
    });
}
// ... 单独更新其他图表
```

**技术细节：**
- 确保DOM元素完全加载后再初始化图表
- 添加try-catch错误处理
- 单独更新每个图表，避免循环问题

---

## 技术栈总结

### 后端技术
- **语言：** C++
- **WebSocket服务器：** 自定义实现
- **图像处理：** OpenCV
- **JSON序列化：** nlohmann/json
- **加密：** OpenSSL (SHA1)

### 前端技术
- **HTML5**
- **CSS3**
- **JavaScript (ES6)**
- **ECharts 5.4.3**

### 通信协议
- **WebSocket** - 实时双向通信
- **HTTP** - 静态文件服务

---

## 关键代码文件

### 后端文件
1. **auto_aim_test.cpp** - 主程序，包含WebSocket服务器初始化和数据广播
2. **websocket_server.hpp** - WebSocket服务器头文件
3. **websocket_server.cpp** - WebSocket服务器实现

### 前端文件
1. **index.html** - 主页面，包含所有波形图表
2. **echarts.min.js** - ECharts库文件

---

## 性能优化

### 1. 数据传输优化
- JPEG质量从95降至60
- 数据大小减少约85%
- 使用Base64编码确保兼容性

### 2. 图表渲染优化
- 禁用动画（animation: false）
- 使用symbol: 'none'减少渲染开销
- 限制数据点数量（默认200点）

### 3. 连接管理优化
- 实现连接监控
- 自动清理断开连接
- 忽略SIGPIPE信号

---

## 测试验证

### 功能测试
1. ✅ 视频流正常显示
2. ✅ 11个波形图表正常显示
3. ✅ 实时数据更新
4. ✅ 交互功能正常（缩放、平移、清除）
5. ✅ 连接稳定性良好

### 性能测试
- FPS：50-80
- 帧延迟：10-25ms
- WebSocket连接稳定

---

## 经验总结

### 成功经验
1. **逐步诊断** - 从基础连接开始，逐步排查问题
2. **日志记录** - 详细的日志帮助快速定位问题
3. **错误处理** - 完善的错误处理提高稳定性
4. **模块化设计** - 独立的WebSocket服务器便于维护

### 教训
1. **避免依赖外部CDN** - 本地化资源提高稳定性
2. **处理边界情况** - 为无检测状态添加默认值
3. **监控程序状态** - 确保服务持续运行
4. **代码审查** - 注意缩进和逻辑错误

---

## 后续改进建议

1. **性能优化**
   - 考虑使用WebGL加速图表渲染
   - 实现数据压缩减少传输量

2. **功能增强**
   - 添加数据导出功能
   - 支持自定义图表配置
   - 添加历史数据回放

3. **用户体验**
   - 添加加载动画
   - 优化移动端显示
   - 添加暗色/亮色主题切换

---

## 附录：常用命令

### 启动服务
```bash
# 启动HTTP服务器
cd /home/pldx/Desktop/pl_vision/web
python3 -m http.server 8000

# 启动auto_aim_test
cd /home/pldx/Desktop/pl_vision
./build/auto_aim_test assets/demo/3m
```

### 调试命令
```bash
# 检查端口占用
lsof -i :8080
lsof -i :8000

# 终止进程
kill -9 <PID>

# 查看日志
tail -f /path/to/logfile
```

### 构建命令
```bash
cd /home/pldx/Desktop/pl_vision
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

---

**文档版本：** 1.0  
**最后更新：** 2026-02-25  
**维护者：** z1396
