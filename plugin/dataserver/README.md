# MuJoCo DataServer Plugin

The DataServer plugin streams structured MuJoCo simulation data (joints, bodies,
sensors, actuators) to external processes through shared memory, and accepts
actuator commands in the reverse direction. It is packaged as a standard
`mujoco.dataserver` plugin so it can be enabled directly from MJCF or runtime
APIs without modifying the MuJoCo core.

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

1. 在仓库根目录运行 CMake：

	 ```bash
	 cmake -B build -S . 
	 cmake --build build --target dataserver
	 ```

2. 编译完成后，`dataserver` 动态库会被安装到
	 `build/bin/<config>/mujoco_plugin/`，MuJoCo 在运行时会自动找到该插件。

3. FlatBuffers 依赖：
	 - 默认使用 `plugin/dataserver/_deps/flatbuffers` 子模块。
	 - 若希望链接系统版本，可在 CMake 配置阶段添加
		 `-DMUJOCO_DATASERVER_USE_SYSTEM_FLATBUFFERS=ON`，并确保 `flatbuffers` 包可被
		 `find_package()` 找到。

## 客户端示例

`shm_client_example.cc` 演示了如何消费数据并回传命令：

```bash
cmake --build build --target shm_client_example
./build/bin/<config>/shm_client_example mujoco_data_0
```

示例流程：

1. 连接到命名共享内存 (`global_monitor` 或自定义名称)。
2. 调用 `ReceiveAllData()` 获取最新的关节/传感器/刚体/执行器状态并打印。
3. 通过 `SendActuatorCommands()` 发送基于正弦波的控制信号。
4. 使用 `WaitForData()` 或 `IsConnected()` 判断共享内存是否可用，便于与实时仿真解耦。

## 扩展接口

- **ServerBase**：`data_type.h` 中定义了抽象类 `ServerBase`，仅包含
	`SendAllData()` 与 `ReceiveActuatorCommands()` 两个纯虚函数。若需要自定义
	传输层（如 TCP/IPC、ZeroMQ、WebSocket 等），实现该接口并在 `DataServer::StartServer()` 中替换 `ShmServer` 即可。
- **数据结构**：`JointData`、`SensorData`、`PoseData`、`ActuatorData` 与
	`MujocoDataFrame` 保持 POD 风格，便于序列化到 FlatBuffers、Cap'n Proto 或自定义格式。
- **模型树导出**：`ModelTreeBuilder`（见 `model_tree.*`）可以将当前 `mjModel`
	转换为树形结构和 MJCF 文本，方便客户端同步模型拓扑信息。
- **控制钩子**：`UpdateActuatorControls()` 会在 `Compute()` 中应用最新控制命令，若需要更多自定义逻辑，可在该函数内扩展。

## 调试与常见问题

- **共享内存命名冲突**：若多个仿真实例并行运行，请通过 `server_args` 指定不同的名称，或关闭 `IS_SINGLETON` 并在参数末尾自动追加实例 ID。
- **clangd 报错**：确保 CMake 生成 `compile_commands.json` 并包含 Windows SDK、MSVC STL 以及 `_deps/flatbuffers/include` 路径，clangd 与 IntelliSense 才能共享一致的编译配置。
- **命令延迟**：如需最小延迟，可将 `async` 置为 `false`，在仿真线程内同步收发；若追求吞吐率，则保持异步并调高 `update_rate_`。

## 进一步工作

- 为常见协议（如 gRPC、ROS2）实现更多 `ServerBase` 后端。
- 在 `shm_client_example` 基础上提供 Python 绑定，方便快速原型控制器。
- 引入数据压缩/筛选策略，降低跨进程带宽占用。
