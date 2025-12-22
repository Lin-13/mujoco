# MuJoCo DataServer Plugin

DataServer 插件通过共享内存将 MuJoCo 仿真数据（关节、刚体、传感器、执行器）流式传输到外部进程,并支持反向接收执行器控制命令。它以标准的 `mujoco.dataserver` 插件形式打包,可直接在 MJCF 中启用或通过运行时 API 加载,无需修改 MuJoCo 核心代码。

## 📑 目录

- [主要功能](#主要功能)
- [配置与使用](#配置与使用)
- [构建说明](#构建说明)
  - [前置条件](#前置条件)
  - [完整构建步骤](#完整构建步骤)
  - [构建产物位置](#构建产物位置)
  - [验证构建](#验证构建)
  - [常见问题](#常见问题)
- [客户端示例](#客户端示例)
- [扩展接口](#扩展接口)
- [性能考虑](#性能考虑)
- [故障排查](#故障排查)

## 主要功能

- **可选数据通道**：通过 `joints`、`bodies`、`sensors`、`actuators` 配置键，选择需要监控或控制的实体，使用 `all` 采集全量信息。
- **实时数据帧**：内部使用 `JointData`、`PoseData`、`SensorData`、`ActuatorData` 结构组合成 `MujocoDataFrame`，便于客户端一次性获取完整状态快照。
- **双向共享内存服务器**：默认的 `ShmServer`/`ShmClient` 组合基于命名共享内存段和跨进程同步原语，实现零拷贝、高吞吐的通信链路。
- **异步推送**：可配置在独立线程以固定频率推送数据并收取控制指令，避免通信延迟阻塞仿真主循环。
- **命令回放**：接收到的执行器命令被映射回仿真 `mjData::ctrl`，支持闭环控制与远程操控。

## 配置与使用

在 MJCF 中添加插件段落即可启用数据服务器：

```xml
<plugin name="mujoco.dataserver" instance="dataserver">
	<config key="server_args" value="global_monitor"/>
	<config key="joints" value="hip_yaw;hip_roll;knee"/>
	<config key="bodies" value="torso;left_foot"/>
	<config key="sensors" value="all"/>
	<config key="actuators" value="all"/>
	<config key="async" value="true"/>
</plugin>
```

可用配置键：

| 键名          | 描述                                                                                                         |
| ------------- | ------------------------------------------------------------------------------------------------------------ |
| `server_args` | 共享内存/服务器参数，使用分号分隔。缺省时会自动生成 `mujoco_data_<instance>` / `mujoco_command_<instance>`。 |
| `joints`      | 要监控的关节列表或 `all`。                                                                                   |
| `bodies`      | 要监控的刚体列表或 `all`。                                                                                   |
| `sensors`     | 要监控的 MuJoCo 传感器或 `all`，会自动跳过 DataServer 自身的传感器条目。                                     |
| `actuators`   | 需要接收控制命令的执行器列表或 `all`。                                                                       |
| `async`       | `true/false`，决定数据服务器是否在独立线程异步运行。关闭后通信会在 `Compute()` 调用中同步进行。              |


### 构建说明

> **💡 快速开始提示**：如果您不想自己编译，可以直接从 [MuJoCo GitHub Releases](https://github.com/Lin-13/mujoco/releases) 下载已编译好的版本。Release 包中已包含 DataServer 插件和 `simulate` 可执行文件以及`shm_client_example`客户端，程序启动时会自动识别并加载插件，无需额外配置。适合想要快速测试插件功能的用户。

#### 前置条件

在开始构建之前，请确保您的系统已安装以下工具：

**Windows 系统：**

- [Visual Studio 2019 或更新版本](https://visualstudio.microsoft.com/)（需包含 C++ 桌面开发工作负载）
- [CMake 3.16 或更新版本](https://cmake.org/download/)
- [Git](https://git-scm.com/download/win)（用于克隆仓库）

**Linux 系统：**

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install build-essential cmake git

# Fedora/RHEL
sudo dnf install gcc-c++ cmake git
```

**macOS 系统：**

```bash
# 安装 Xcode Command Line Tools
xcode-select --install

# 安装 CMake（使用 Homebrew）
brew install cmake
```

#### 完整构建步骤

##### 步骤 1: 克隆或导航到 MuJoCo 仓库

```bash
# 如果还没有克隆仓库
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco

# 如果已经有仓库，直接进入根目录
cd /path/to/mujoco
```

**重要提示**：DataServer 插件是 MuJoCo 仓库的一部分，必须从 MuJoCo 项目根目录构建，而不能单独构建插件目录。

##### 步骤 2: 配置 CMake 构建

在 MuJoCo **根目录**运行：

```bash
# Windows (PowerShell)
cmake -B build -S .

# Linux/macOS
cmake -B build -S .
```

这一步会：

- 创建 `build` 目录
- 检测您的编译器和系统配置
- 自动下载和配置 FlatBuffers（如果使用默认设置）
- 生成构建文件

**可选配置参数：**

如果希望使用系统安装的 FlatBuffers 而非内置版本：

```bash
cmake -B build -S . -DMUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS=ON
```

> **注意**：使用系统 FlatBuffers 前需先安装：
>
> - **Linux**: `sudo apt install libflatbuffers-dev`
> - **Windows**: 使用 vcpkg 安装 `vcpkg install flatbuffers:x64-windows`
> - **macOS**: `brew install flatbuffers`

**请注意 FlatBuffers 的版本号，如有必要可以重新生成 data_frame.fbs**

**使用 flatc 重新生成代码：**

如果需要修改数据结构或更新 FlatBuffers 版本，可以使用 flatc 编译器重新生成 C++ 代码：

```bash
# 进入 data_frame 目录
cd plugin/dataserver/data_frame

# 使用 flatc 生成 C++ 头文件
flatc --cpp data_frame.fbs

# 生成的文件会是 data_frame_generated.h
# 确认生成成功
ls -l data_frame_generated.h
```

##### 步骤 3: 编译插件

```bash
# 仅编译 dataserver 插件（推荐首次构建时使用）
cmake --build build --target dataserver

# 或者编译所有插件和主库
cmake --build build
```

**Windows 用户**可以指定构建配置：

```bash
# Release 构建（生产使用）
cmake --build build --config Release --target dataserver

# Debug 构建（开发调试）
cmake --build build --config Debug --target dataserver
```

##### 步骤 4: 编译客户端库和示例程序

```bash
# 编译客户端库
cmake --build build --target dataclient

# 编译示例程序
cmake --build build --target shm_client_example
```

##### 步骤 5: 安装（可选）

将插件和库安装到标准位置：

```bash
cmake --build build --target install
```

#### 构建产物位置

编译成功后，您可以在以下位置找到生成的文件：

**插件库：**

- **Windows**: `build/bin/Release/mujoco_plugin/dataserver.dll`（或 `Debug/` 目录）
- **Linux**: `build/bin/mujoco_plugin/libdataserver.so`
- **macOS**: `build/bin/mujoco_plugin/libdataserver.dylib`

**客户端库：**

- **Windows**: `build/lib/Release/dataclient.lib`
- **Linux/macOS**: `build/lib/libdataclient.a`

**示例程序：**

- **Windows**: `build/bin/Release/shm_client_example.exe`
- **Linux/macOS**: `build/bin/shm_client_example`

**安装后的文件结构**（如果执行了 `cmake --build build --target install`）：

``` shell
build/install/
├── bin/
│   ├── shm_client_example
│   └── mujoco_plugin/
│       └── dataserver.[dll|so|dylib]
├── lib/
│   └── libdataclient.a
└── include/
    └── dataserver/
        ├── data_type.h
        ├── shm_manager.h
        └── shm_client.h
```

#### 验证构建

构建完成后，验证插件是否正常工作：

```bash
# 方法 1: 检查文件是否存在
# Windows
dir build\bin\Release\mujoco_plugin\dataserver.dll

# Linux/macOS
ls -l build/bin/mujoco_plugin/libdataserver.so

# 方法 2: 运行示例程序（需先启动包含 dataserver 插件的 MuJoCo 仿真）
# Windows
.\build\bin\Release\shm_client_example.exe

# Linux/macOS
./build/bin/shm_client_example
```

#### 常见问题

**问题 1：找不到 mujoco 库**

```
CMake Error: Could not find mujoco library
```
**解决方案**：确保您在 MuJoCo 项目的根目录下运行 CMake 命令，而不是在 `plugin/dataserver` 子目录。

**问题 2：FlatBuffers 相关错误**

```
Could not find FlatBuffers
```
**解决方案**：

- 默认情况下应自动使用内置的 FlatBuffers。确认 `plugin/dataserver/_deps/flatbuffers` 目录存在。
- 如果使用系统 FlatBuffers，确保已正确安装并在 CMake 命令中指定路径。

**问题 3：Windows 上找不到 MSVC 编译器**

``` shell
No CMAKE_CXX_COMPILER could be found
```

**解决方案**：

1. 安装 Visual Studio 2019 或更新版本
2. 在安装过程中确保选择"使用 C++ 的桌面开发"工作负载
3. 使用"Developer Command Prompt for VS"或"Developer PowerShell for VS"运行 CMake 命令，或者将 CMake 添加到环境变量

**问题 4：权限错误（Linux/macOS）**

```
Permission denied
```

**解决方案**：不要使用 `sudo` 运行构建命令。确保您对项目目录有写入权限：

```bash
chmod -R u+w /path/to/mujoco
```

#### FlatBuffers 依赖说明

DataServer 使用 [FlatBuffers](https://google.github.io/flatbuffers/) 进行高效的数据序列化。默认配置会自动处理此依赖：

- **默认方式**（推荐）：使用插件内置的 FlatBuffers 子模块（`MUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS=OFF`）
  - 优点：无需手动安装，构建自动处理
  - 缺点：首次构建时间稍长

- **系统安装方式**：使用系统已安装的 FlatBuffers（需要 `-DMUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS=ON`）
  - 优点：如果系统已有 FlatBuffers，可复用现有安装
  - 缺点：需要确保版本兼容性

## 客户端示例

### 快速开始

`shm_client_example.cc` 演示了如何通过共享内存与 MuJoCo 仿真进行数据交互。

#### 运行示例

**步骤 1: 准备测试环境**
首先，您需要一个启用了 DataServer 插件的 MuJoCo 模型。创建一个测试 MJCF 文件（例如 `test_dataserver.xml`）：

```xml
<mujoco>
  <compiler angle="degree"/>
  
  <option timestep="0.002"/>
  
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1"/>
    
    <body name="box" pos="0 0 0.5">
      <joint name="free_joint" type="free"/>
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
      <site name="box_site" pos="0 0 0"/>
    </body>
  </worldbody>
  
  <sensor>
    <framepos name="box_pos" objtype="site" objname="box_site"/>
    <framequat name="box_quat" objtype="site" objname="box_site"/>
  </sensor>
  
  <plugin name="mujoco.dataserver" instance="my_dataserver">
    <config key="server_args" value="test_shm"/>
    <config key="joints" value="all"/>
    <config key="bodies" value="all"/>
    <config key="sensors" value="all"/>
    <config key="actuators" value="all"/>
    <config key="async" value="true"/>
  </plugin>
</mujoco>
```

**步骤 2: 启动 MuJoCo 仿真**
使用 MuJoCo 的 `simulate` 工具或您自己的程序加载上述模型：

```bash
# 如果已经构建了 simulate
./build/bin/simulate test_dataserver.xml


**步骤 3: 运行客户端示例**
在另一个终端窗口运行示例程序：

```bash
# Windows
.\build\bin\Release\shm_client_example.exe

# Linux/macOS
./build/bin/shm_client_example
# 也可以在/build/install/bin中启动
```

### 示例程序工作流程

示例程序的核心流程包括：

1. **连接共享内存**

   ```cpp
   ShmClient client("test_shm");  // 连接名为 "test_shm" 的共享内存
   if (!client.IsConnected()) {
       std::cerr << "Failed to connect to shared memory\n";
       return 1;
   }
   ```

2. **接收仿真数据**

   ```cpp
   MujocoDataFrame frame = client.ReceiveAllData();
   
   // 访问关节数据
   for (const auto& joint : frame.joints) {
       std::cout << "Joint " << joint.id << ": pos=" << joint.position
                 << ", vel=" << joint.velocity << "\n";
   }
   
   // 访问传感器数据
   for (const auto& sensor : frame.sensors) {
       std::cout << "Sensor " << sensor.id << ": ";
       for (double value : sensor.data) {
           std::cout << value << " ";
       }
       std::cout << "\n";
   }
   
   // 访问刚体位姿
   for (const auto& body : frame.bodies) {
       std::cout << "Body " << body.id << ": pos=("
                 << body.position[0] << ", "
                 << body.position[1] << ", "
                 << body.position[2] << ")\n";
   }
   ```

3. **发送控制命令**

   ```cpp
   std::vector<ActuatorCommand> commands;
   for (size_t i = 0; i < frame.actuators.size(); ++i) {
       ActuatorCommand cmd;
       cmd.actuator_id = frame.actuators[i].id;
       // 生成正弦波控制信号
       cmd.control = std::sin(time * 2.0 + i * 0.5);
       commands.push_back(cmd);
   }
   client.SendActuatorCommands(commands);
   ```

4. **同步与轮询**

   ```cpp
   // 等待新数据到达
   if (client.WaitForData(1000)) {  // 超时 1000ms
       // 数据已更新，可以读取
   }
   
   // 或者使用非阻塞轮询
   while (client.IsConnected()) {
       frame = client.ReceiveAllData();
       // 处理数据...
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
   }
   ```

### 自定义客户端开发

要在自己的项目中使用 DataServer 客户端库：

**CMakeLists.txt 配置：**

```cmake
# 添加 dataclient 库
add_executable(my_client my_client.cc)
target_link_libraries(my_client PRIVATE dataclient)

# 如果需要手动指定路径
target_include_directories(my_client PRIVATE 
    ${CMAKE_SOURCE_DIR}/plugin/dataserver
)
```

**基础客户端模板：**

```cpp
#include "shm_client.h"
#include <iostream>

int main() {
    // 1. 创建客户端（连接到插件配置的 server_args）
    mujoco::dataserver::ShmClient client("test_shm");
    
    if (!client.IsConnected()) {
        std::cerr << "无法连接到共享内存。请确保：\n"
                  << "  - MuJoCo 仿真正在运行\n"
                  << "  - MJCF 中已启用 dataserver 插件\n"
                  << "  - server_args 名称匹配\n";
        return 1;
    }
    
    std::cout << "成功连接到 MuJoCo DataServer!\n";
    
    // 2. 主循环：读取数据并发送命令
    while (client.IsConnected()) {
        // 接收最新数据
        auto frame = client.ReceiveAllData();
        
        // TODO: 处理数据，实现您的控制算法
        
        // 准备并发送命令
        std::vector<ActuatorCommand> commands;
        // TODO: 根据状态生成控制命令
        
        if (!commands.empty()) {
            client.SendActuatorCommands(commands);
        }
        
        // 控制循环频率
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "连接已断开\n";
    return 0;
}
```

### 多客户端支持

DataServer 支持多个客户端同时连接到同一共享内存：

```cpp
// 客户端 1: 监控数据
ShmClient monitor("global_monitor");
while (monitor.IsConnected()) {
    auto frame = monitor.ReceiveAllData();
    LogData(frame);  // 记录日志
}

// 客户端 2: 发送控制（在另一个进程中）
ShmClient controller("global_monitor");
while (controller.IsConnected()) {
    auto commands = ComputeControl();
    controller.SendActuatorCommands(commands);
}
```

**注意事项：**

- 多个客户端可以同时读取数据
- 控制命令会被最后发送的客户端覆盖
- 确保不同控制客户端之间有适当的协调机制

## 扩展接口

DataServer 提供了灵活的扩展机制,允许自定义传输层和数据处理逻辑。

### 自定义传输层

**ServerBase 抽象接口**
`data_type.h` 中定义了抽象类 `ServerBase`,包含两个核心方法：

```cpp
class ServerBase {
public:
    virtual ~ServerBase() = default;
    
    // 发送仿真数据到客户端
    virtual void SendAllData(const MujocoDataFrame& frame) = 0;
    
    // 从客户端接收执行器命令
    virtual std::vector<ActuatorCommand> ReceiveActuatorCommands() = 0;
};
```

**实现自定义传输层示例（TCP）**

```cpp
#include "data_type.h"
#include <boost/asio.hpp>

class TcpServer : public ServerBase {
private:
    boost::asio::ip::tcp::socket socket_;
    
public:
    void SendAllData(const MujocoDataFrame& frame) override {
        // 序列化数据帧
        std::vector<uint8_t> buffer = SerializeFrame(frame);
        // 通过 TCP 发送
        boost::asio::write(socket_, boost::asio::buffer(buffer));
    }
    
    std::vector<ActuatorCommand> ReceiveActuatorCommands() override {
        // 从 TCP 接收命令
        std::vector<uint8_t> buffer(1024);
        size_t len = socket_.read_some(boost::asio::buffer(buffer));
        // 反序列化命令
        return DeserializeCommands(buffer, len);
    }
};
```

**在 DataServer 中使用自定义传输层**
修改 `data_server.cc` 中的 `StartServer()` 方法：

```cpp
void DataServer::StartServer() {
    // 替换默认的 ShmServer
    server_ = std::make_unique<TcpServer>(/* 参数 */);
    // ... 其余逻辑
}
```

### 数据结构说明

所有数据结构都是 POD（Plain Old Data）类型,便于序列化：

**JointData** - 关节状态

```cpp
struct JointData {
    int id;                    // 关节 ID
    double position;           // 位置
    double velocity;           // 速度
    double acceleration;       // 加速度
    std::array<double, 3> axis; // 关节轴向量
};
```

**PoseData** - 刚体位姿

```cpp
struct PoseData {
    int id;                        // 刚体 ID
    std::array<double, 3> position;    // 位置 (x, y, z)
    std::array<double, 4> quaternion;  // 四元数 (w, x, y, z)
    std::array<double, 3> linear_vel;  // 线速度
    std::array<double, 3> angular_vel; // 角速度
};
```

**SensorData** - 传感器数据

```cpp
struct SensorData {
    int id;                     // 传感器 ID
    std::vector<double> data;   // 传感器输出（维度可变）
};
```

**ActuatorData** - 执行器状态

```cpp
struct ActuatorData {
    int id;              // 执行器 ID
    double length;       // 当前长度
    double velocity;     // 当前速度
    double force;        // 当前作用力
};
```

**MujocoDataFrame** - 完整数据帧

```cpp
struct MujocoDataFrame {
    double time;                          // 仿真时间
    std::vector<JointData> joints;        // 所有关节
    std::vector<PoseData> bodies;         // 所有刚体
    std::vector<SensorData> sensors;      // 所有传感器
    std::vector<ActuatorData> actuators;  // 所有执行器
};
```

### 控制钩子扩展

在 `data_server.cc` 中自定义控制逻辑：

```cpp
void DataServer::UpdateActuatorControls(mjData* data) {
    auto commands = server_->ReceiveActuatorCommands();
    
    for (const auto& cmd : commands) {
        // 基础控制映射
        data->ctrl[cmd.actuator_id] = cmd.control;
        
        // 添加自定义逻辑：
        // 1. 限幅保护
        double max_force = 100.0;
        data->ctrl[cmd.actuator_id] = std::clamp(
            cmd.control, -max_force, max_force
        );
        
        // 2. 安全模式检测
        if (safety_mode_active_) {
            data->ctrl[cmd.actuator_id] = 0.0;
        }
        
        // 3. 记录命令历史
        command_history_.push_back({cmd, data->time});
    }
}
```

## 性能考虑

### 同步 vs 异步模式

**同步模式** (`async="false"`)

- **优点**：
  - 零延迟：数据传输在仿真步内完成
  - 精确时序：命令立即应用于当前时间步
  - 简单调试：无多线程复杂性
- **缺点**：
  - 通信延迟会阻塞仿真循环
  - 不适合高频率仿真（> 1kHz）
- **适用场景**：硬件在环仿真、实时控制器测试

**异步模式** (`async="true"`)

- **优点**：
  - 高吞吐：仿真与通信并行
  - 不阻塞仿真主循环
  - 适合长时间运行
- **缺点**：
  - 命令应用延迟 1-2 个时间步
  - 需要线程同步机制
- **适用场景**：数据记录、离线分析、高频率仿真

### 数据选择优化

只采集必要的数据以减少开销：

```xml
<!-- 不推荐：采集所有数据 -->
<config key="joints" value="all"/>
<config key="bodies" value="all"/>
<config key="sensors" value="all"/>
<config key="actuators" value="all"/>

<!-- 推荐：仅采集需要的数据 -->
<config key="joints" value="joints1;joints2;joints3"/>
<config key="sensors" value="sensor1"/>
<config key="actuators" value="motor1;motor2"/>
```

### 共享内存优化

```cpp
// 在客户端减少轮询频率
while (client.IsConnected()) {
    if (client.WaitForData(10)) {  // 10ms 超时
        auto frame = client.ReceiveAllData();
        ProcessFrame(frame);
    }
}

// 而非忙等待
while (client.IsConnected()) {
    auto frame = client.ReceiveAllData();  // 占用 CPU
    ProcessFrame(frame);
}
```

## 故障排查

### 问题诊断清单

当遇到问题时，按以下步骤检查：

#### 1. 插件是否正确加载？

**检查方法：**

```bash
# 确认插件文件存在
# Windows
dir build/install/bin/mujoco_plugin/dataserver.dll

# Linux/macOS  
ls -l build/install/bin/mujoco_plugin/libdataserver.so
```

**MuJoCo 日志：**
启动 MuJoCo 时查看控制台输出：

``` shell
Plugin registered: mujoco.dataserver
DataServer instance 'my_dataserver' initialized
```

如果没有看到这些信息，检查：

- MJCF 文件中 `<plugin>` 标签是否正确
- `MUJOCO_PLUGIN_DIR` 环境变量是否设置
- 插件文件是否在正确的目录

#### 2. 共享内存连接失败

**错误现象：**

``` shell
Failed to connect to shared memory
ShmClient::IsConnected() returns false
```

**解决步骤：**

1. **确认 server_args 名称匹配**

   ```xml
   <!-- MJCF 中 -->
   <config key="server_args" value="test_shm"/>
   ```

   ```cpp
   // 客户端中
   ShmClient client("test_shm");  // 名称必须一致
   ```

2. **检查 MuJoCo 是否正在运行**
   - 仿真必须先启动，客户端才能连接
   - 仿真停止后，共享内存会被清理

3. **权限问题（Linux/macOS）**

   ```bash
   # 查看共享内存
   ls -l /dev/shm/<your_server_args>
   
   # 如有权限问题，清理旧的共享内存
   rm /dev/shm/<your_server_args>
   ```

4. **Windows 命名冲突**

   ```bash
   # 使用 Process Explorer 查看命名对象
   # 或重启计算机清理所有共享内存
   ```

#### 3. 数据更新不及时

**现象**：客户端收到的数据总是旧的或不变化

**检查项：**

1. **async 配置**

   ```xml
   <!-- 确保 async 模式开启 -->
   <config key="async" value="true"/>
   ```

2. **客户端轮询频率**

   ```cpp
   // 使用 WaitForData 而非忙等待,设置timeout
   if (client.WaitForData(100)) {
       auto frame = client.ReceiveAllData();
   }
   ```

3. **仿真是否正在运行**
   - 检查仿真是否暂停
   - 确认时间步正在推进

#### 4. 控制命令不生效

**现象**：发送的命令没有作用于仿真

**检查项：**

1. **执行器配置**

   ```xml
   <!-- 确保配置了 actuators -->
   <config key="actuators" value="all"/>
   <!-- 或指定具体执行器 -->
   <config key="actuators" value="motor1;motor2"/>
   ```

2. **命令格式正确**

   ```cpp
   std::unordered_map<std::string, double> commands;
   
   commands["actuator_name"] = actuator_value;
   client.SendActuatorCommands(commands);
   ```

3. **执行器 ID 映射**

   ```cpp
   // 先获取执行器信息
   auto frame = client.ReceiveAllData();
   for (const auto& actuator : frame.actuators) {
       std::cout << "Actuator ID: " << actuator.id << "\n";
   }
   ```

### 调试技巧

#### 启用详细日志

在 `data_server.cc` 中添加调试输出：

```cpp
void DataServer::Compute(const mjModel* m, mjData* d, int instance) {
    std::cout << "[DataServer] time=" << d->time 
              << ", nq=" << m->nq << "\n";
    // ... 其余代码
}
```

#### 共享内存检查工具

**Linux:**

```bash
# 查看所有共享内存段
ipcs -m

# 查看特定共享内存详情
ls -l /dev/shm

# 清理所有共享内存
rm /dev/shm/*
```

## 进一步工作

- **更多传输层后端**：实现 gRPC、ROS2、ZeroMQ 等常见协议的 `ServerBase` 后端
- **Python**：类似 `shm_client_example` 提供 Python方法 ，方便快速原型开发
- **数据压缩**：引入数据筛选和压缩策略，降低跨进程带宽占用
- **远程调试工具**：开发 GUI 工具用于实时监控和控制
- **性能分析**：集成性能计数器，分析通信开销

## 参考资源

- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [FlatBuffers 文档](https://google.github.io/flatbuffers/)
- [共享内存编程（POSIX）](https://man7.org/linux/man-pages/man7/shm_overview.7.html)
