// 基于共享内存的数据接收client
#pragma once
#include "data_type.h"
#include "shm_manager.h"
#include <memory>
#include <vector>

class ShmClient {
public:
  ShmClient(const std::vector<std::string> &shm_names, size_t shm_size);
  ~ShmClient();
  bool Initialize();
  bool IsConnected() const {
    bool check_data = data_ != nullptr && command_ != nullptr &&
                      data_sync_ != nullptr && command_sync_ != nullptr;
    if (!check_data) {
      return false;
    }
    bool check_data_lock = data_sync_->lock();
    if (check_data_lock) {
      data_sync_->unlock();
    }
    bool check_command_lock = command_sync_->lock();
    if (check_command_lock) {
      command_sync_->unlock();
    }
    return check_data && check_data_lock && check_command_lock;
  }
  // 接收所有数据（关节、传感器、身体、执行器）
  bool ReceiveAllData(std::vector<JointData> &joint_data,
                      std::vector<SensorData> &sensor_data,
                      std::vector<PoseData> &body_data,
                      std::vector<ActuatorData> &actuator_data);

  // 发送执行器控制命令
  void SendActuatorCommands(
      const std::unordered_map<std::string, double> &actuator_commands);

  // 等待新数据（带超时）
  bool WaitForData(int timeout_ms = 1000);

private:
  std::shared_ptr<ShmManager> data_, command_;
  std::shared_ptr<ShmSync> data_sync_, command_sync_;
  std::vector<std::string> shm_names_;
  size_t shm_size_;
};
