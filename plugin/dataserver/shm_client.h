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
  /**
   * @brief
   * 接收所有数据，is_locked表示调用时是否已经加锁,避免重复加锁，实现与wait的同步
   *
   * @param data_frame
   * @param is_locked
   * @return true
   * @return false
   */
  bool ReceiveAllData(MujocoDataFrame &data_frame, bool is_locked = false);

  // 发送执行器控制命令
  void SendActuatorCommands(
      const std::unordered_map<std::string, double> &actuator_commands);

  // 等待新数据（带超时）
  bool WaitForData(int timeout_ms = 1000);
  bool WaitForData(MujocoDataFrame &data_frame, int timeout_ms = 1000);

private:
  std::shared_ptr<ShmManager> data_, command_;
  std::shared_ptr<ShmSync> data_sync_, command_sync_;
  std::vector<std::string> shm_names_;
  size_t shm_size_;
  uint64_t command_frame_id_ = 0; // 命令帧ID计数器
};
