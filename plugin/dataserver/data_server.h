// data_server.h
#ifndef DATA_SERVER_H
#define DATA_SERVER_H

#include "model_tree.h"
#include <memory>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>
namespace mujoco::plugin::dataserver {

class DataServer {
public:
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
  DataServer(int port, int instance);
  ~DataServer();
  static std::unique_ptr<DataServer> Create(const mjModel *m, int instance);
  static std::unordered_map<void *, int> INSTANCE_LIST;
  void Reset(mjtNum *plugin_state);
  void Compute(const mjModel *m, mjData *d, int instance);

  static void RegisterPlugin();

private:
  int port_;
  int instance_;
  // 模型树根节点
  std::shared_ptr<BodyNode> model_tree_root_;
  std::string model_mjcf_;
  // 关节传感器 joint_ids_和joint_names_弃用，统一使用joint_data_
  // buffer在InitializeJointBuffer中初始化
  std::vector<int> joint_ids_;           // 监控的关节ID
  std::vector<std::string> joint_names_; // 关节名称
  std::vector<JointData> joint_data_;    // 关节状态缓冲区
  // 关节执行器
  std::vector<int> actuator_ids_;           // 控制的执行器ID
  std::vector<std::string> actuator_names_; // 执行器名称
  std::vector<double> actuator_gains_;      // 执行器增益
  // 执行器命令缓冲区,
  // 使用map存储名称到命令的映射,不需要包含所有actuator_names_的键值对
  std::unordered_map<std::string, double> actuator_commands_;

  // body buffer index,用于快速访问body数据
  std::vector<int> body_ids_;           // 监控的身体ID
  std::vector<std::string> body_names_; // 身体名称
  std::vector<int> body_xpos_adr_;  // 身体位置在xpos中的起始地址（3*body_id）
  std::vector<int> body_xquat_adr_; // 身体姿态在xquat中的起始地址（4*body_id）
  std::vector<int> body_xvelp_adr_; // 身体线速度在cvel中的地址（6*body_id）
  std::vector<int> body_xvelr_adr_; // 身体角速度在cvel中的地址（6*body_id + 3）
  // body state buffer
  std::vector<double> body_positions_;
  std::vector<double> body_orientations_;
  std::vector<double> body_velocities_;
  // sensor ：mjcf中定义的传感器buffer
  std::vector<SensorData> sensor_data_;
  // 其他成员变量
  bool is_initialized_ = false;

  // JointSensor 和 Actuator
  std::vector<double> GetJointPositions();
  int GetPluginInstance() { return instance_; }
  void InitializeJointDataBuffer(const mjModel *m, const char *joints_config);
  void InitializeJointActuators(const mjModel *m, const char *actuators_config);
  void GetJointData(const mjModel *m, const mjData *d);
  void UpdateActuatorControls(const mjModel *m, mjData *d);
  // Body相关数据
  void InitializeBodyDataBuffer(const mjModel *m, const char *bodies_config);
  void GetBodyPoseData(const mjModel *m, const mjData *d);
  int GetNumBodyData() const;
  // Sensor 相关
  void InitializeSensorDataBuffer(const mjModel *m, const char *sensors_config);
  void GetSensorData(const mjModel *m, const mjData *d);
  int GetNumSensorData() const;
  // camera
  void GetCameraSensorData(const mjModel *m, const mjData *d);
  // 网络通信方法
  void StartServer();
  void SendData();
  void ReceiveControlCommands();
};

} // namespace mujoco::plugin::dataserver

#endif // DATA_SERVER_H