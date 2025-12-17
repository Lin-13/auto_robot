# Auto Robot

一个基于C++的机器人控制与标定系统，专为AUBO机器人设计，集成了力传感器、视觉标定、运动控制等功能。

## 项目简介

本项目是一个综合性的机器人控制平台，主要功能包括：
- AUBO机器人的控制与通信
- 手眼标定（Eye-to-Hand Calibration）
- 力传感器集成与力控制
- OptiTrack运动捕捉系统集成
- 混合力/位置控制
- 实时上下文监控（基于gRPC）
- MuJoCo物理仿真支持

## 主要特性

### 机器人控制
- 支持AUBO机器人控制接口
- 正/逆运动学计算（基于KDL库）
- 轨迹规划与执行
- 双臂机器人协同控制

### 传感器集成
- 六维力传感器数据采集与处理
- 力控制算法实现
- 力传感器标定功能
- OptiTrack运动捕捉系统接口

### 视觉标定
- 基于ArUco标记的手眼标定
- 棋盘格相机标定
- Intel RealSense相机支持
- 手眼标定工具（支持OptiTrack）

### 实时监控
- 基于gRPC的实时数据监控
- Python可视化工具
- 上下文数据记录与分析

## 系统要求

### 编译器
- GCC 12+ 或 Clang (支持C++20标准)
- CMake 3.15+

### 依赖库

#### 必需依赖
- **OpenCV 4**: 图像处理与视觉算法
- **Eigen3**: 线性代数库
- **fmt**: 格式化输出库
- **LibXml2**: XML配置文件解析
- **MuJoCo**: 物理仿真引擎
- **gRPC**: 远程过程调用框架
- **SDL2**: 游戏手柄输入支持
- **Orocos-KDL**: 运动学与动力学库
- **GTest**: 单元测试框架

#### 可选依赖
- **Intel RealSense SDK**: 深度相机支持
- **VRPN**: OptiTrack运动捕捉系统支持
- **AUBO Robot SDK**: AUBO机器人控制（位于dependents/robotSDK）

## 编译与安装

### 1. 安装依赖

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake \
    libeigen3-dev libfmt-dev libxml2-dev \
    libsdl2-dev libgtest-dev

# 安装OpenCV 4（需要自行编译或通过包管理器安装）
# 安装MuJoCo（参考官方文档）
# 安装gRPC（参考官方文档）
```

### 2. 配置CMake

编辑 `CMakeLists.txt` 中的路径配置：

```cmake
# 修改为你的OpenCV安装路径，通常可以通过以下命令找到：
# find /usr/local -name "OpenCVConfig.cmake" 或 find /usr -name "OpenCVConfig.cmake"
set(OpenCV4_DIR "/path/to/your/opencv4/cmake")  # 替换为实际的OpenCV4路径
```

### 3. 编译项目

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

### 4. 安装

```bash
make install
```

编译产物将安装到 `install/` 目录。

## 使用说明

### 可执行程序

#### 标定工具
- `aruco_detect`: ArUco标记检测
- `calib`: 棋盘格相机标定
- `calib_handeye`: 手眼标定（基于RealSense）
- `calib_handeye_optitrack`: 手眼标定（基于OptiTrack）

#### 控制程序
- `OptitrackControl`: 基于OptiTrack的机器人控制
- `FTControl`: 力传感器控制测试
- `HybrideControlTest`: 混合力/位置控制测试

#### 测试程序
- `AuboTests`: AUBO机器人功能测试
- `FTSensorTest`: 力传感器测试
- `OptitrackTests`: OptiTrack系统测试
- `MatrixTests`: 矩阵运算单元测试

### Python工具脚本

项目包含多个Python工具脚本（位于 `scripts/` 目录）：

```bash
cd scripts

# 安装Python依赖（项目使用uv包管理器）
uv sync

# 运行上下文监控GUI
python context_monitor_gui.py

# 数据可视化
python signal_plot.py
python read_pose.py
```

主要Python工具：
- `context_monitor_gui.py`: 实时监控GUI界面
- `context_monitor.py`: 上下文监控客户端
- `signal_plot.py`: 信号数据可视化
- `read_pose.py`: 机器人位姿数据读取
- `read_optitrack.py`: OptiTrack数据读取

## 项目结构

```
auto_robot/
├── CMakeLists.txt          # 主CMake配置文件
├── include/                # 头文件目录
│   ├── aubo/              # AUBO机器人接口
│   ├── robot_interface/   # 通用机器人接口
│   ├── ft_sensor/         # 力传感器
│   ├── optitrack/         # OptiTrack接口
│   ├── context_monitor/   # 上下文监控
│   ├── hybrid_control/    # 混合控制
│   └── utils/             # 工具函数
├── src/                    # 源代码目录
│   ├── aubo/              # AUBO机器人实现
│   ├── robot_interface/   # 机器人接口实现
│   ├── ft_sensor/         # 力传感器实现
│   ├── calib_handeye/     # 手眼标定
│   ├── calib_chess/       # 棋盘格标定
│   ├── context_monitor/   # 监控服务实现
│   └── utils/             # 工具函数实现
├── test/                   # 测试代码
├── scripts/                # Python工具脚本
├── simulate/               # MuJoCo仿真相关
├── dependents/             # 第三方依赖
│   ├── robotSDK/          # AUBO机器人SDK
│   └── log4cplus/         # 日志库
└── README.md              # 本文件
```

## 运行测试

```bash
cd build

# 运行所有测试
ctest

# 运行特定测试
./MatrixTests
./AuboTests
./RobotInterface
```

## 配置说明

### 机器人IP配置
在相应的源文件中修改机器人IP地址：
```cpp
static const char *SERVER_HOST_left = "192.168.1.101";
static const char *SERVER_HOST_right = "192.168.1.131";
```

### 标定数据路径
标定数据通常存储在以下位置：
- `optitrack_handeye_left/T_bc.txt`: 左机器人基座变换
- `optitrack_handeye_left/T_et.txt`: 左机器人末端变换
- 手眼标定结果等其他标定文件

## 开发指南

### 代码风格
- 使用C++20标准
- 遵循现有代码的命名约定
- 使用Eigen进行矩阵运算
- 使用fmt库进行格式化输出

### 添加新功能
1. 在 `include/` 和 `src/` 相应目录下添加头文件和源文件
2. 在 `CMakeLists.txt` 中添加库或可执行文件定义
3. 编写单元测试（位于 `test/` 目录）
4. 更新文档

## 常见问题

### 编译错误
- 确保所有依赖库已正确安装
- 检查CMakeLists.txt中的路径配置
- 确保编译器支持C++20

### 运行时错误
- 检查机器人IP地址配置
- 确保AUBO Robot SDK库正确链接
- 验证传感器连接状态

## 贡献

欢迎提交Issue和Pull Request来改进本项目。

## 联系方式

如有问题或建议，请通过GitHub Issues联系。

---

*本文档由 GitHub Copilot 工具自动生成*
