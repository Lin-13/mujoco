// 定义了管理关节、传感器和身体数据的结构体，用于序列化和反序列化
#pragma once
// #include <mujoco/mujoco.h>
typedef double mjtNum; // 避免显式的mujoco依赖，增加代码移植能力
#include <string>
#include <unordered_map>
#include <vector>
struct JointData {
  std::string name;
  int id;
  std::vector<mjtNum> positions;  // 可能多个值
  std::vector<mjtNum> velocities; // 可能多个值
  int joint_type;
};
struct SensorData {
  std::string name;
  int id;
  std::vector<mjtNum> values;
};
struct PoseData {
  std::string name;
  int id;
  mjtNum position[3];
  mjtNum orientation[4];
  mjtNum linear_velocity[3];
  mjtNum angular_velocity[3];
};
struct ActuatorData {
  std::string name;
  int id;
  double data;
};
struct MujocoDataFrame {
  // header
  std::string desctrption;
  uint64_t timestamp; // 微秒时间戳
  bool is_valid;
  uint64_t frame_id;
  uint64_t req_frame_id; // 对应的请求frame_id，用于CS架构中匹配请求-响应
  double sim_time;
  // data
  std::vector<JointData> joints;
  std::vector<SensorData> sensors;
  std::vector<PoseData> bodies;
  std::vector<ActuatorData> actuators;
};
struct MujocoCommandFrame {
  std::unordered_map<std::string, double> commands;
  uint64_t timestamp;
  uint64_t frame_id; // 命令帧ID，用于CS架构中的请求标识
};
class ServerBase {
public:
  virtual ~ServerBase() = default;
  virtual void ReceiveActuatorCommands(MujocoCommandFrame &command_frame) = 0;
  virtual void SendAllData(const MujocoDataFrame &data_frame) = 0;
};