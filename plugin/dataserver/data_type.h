// 定义了管理关节、传感器和身体数据的结构体，用于序列化和反序列化
#pragma once
#include <mujoco/mujoco.h>
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
class ServerBase {
public:
  virtual ~ServerBase() = default;
  virtual void SendJointData(const std::vector<JointData> &joint_data) = 0;
  virtual void SendSensorData(const std::vector<SensorData> &sensor_data) = 0;
  virtual void SendBodyData(const std::vector<PoseData> &body_data) = 0;
  virtual void ReceiveActuatorCommands(
      std::unordered_map<std::string, double> &actuator_commands) = 0;
};