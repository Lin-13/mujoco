// 基于共享内存的数据转发server
#pragma once
#include "data_type.h"
#include "shm_manager.h"
#include <memory>
class ShmServer : public ServerBase {
public:
  ShmServer(const std::vector<std::string> &shm_names, size_t shm_size);
  ~ShmServer() override;
  void ReceiveActuatorCommands(
      std::unordered_map<std::string, double> &actuator_commands) override;
  void SendAllData(const std::vector<JointData> &joint_data,
                   const std::vector<SensorData> &sensor_data,
                   const std::vector<PoseData> &body_data) override;

private:
  std::shared_ptr<ShmManager> data_, command_;
  std::shared_ptr<ShmSync> data_sync_, command_sync_;
  bool shm_destroyed_when_done_ = true;
  std::string shm_name_;
  size_t shm_size_;
  // 其他成员变量，如同步机制等
};