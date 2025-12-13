#include "shm_client.h"
#include "data_frame/command_frame_generated.h"
#include "data_frame/data_frame_generated.h"
#include <chrono>
#include <cstring>
#include <iostream>

ShmClient::ShmClient(const std::vector<std::string> &shm_names, size_t shm_size)
    : shm_names_(shm_names), shm_size_(shm_size) {}
bool ShmClient::Initialize() {
  // 客户端以只读模式打开数据共享内存
  if (shm_names_.size() == 1) {
    data_ = std::make_shared<ShmManager>(shm_names_[0] + "_data", shm_size_);
    command_ =
        std::make_shared<ShmManager>(shm_names_[0] + "_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names_[0] + "_data_sync", false);
    command_sync_ =
        std::make_shared<ShmSync>(shm_names_[0] + "_command_sync", false);
  } else if (shm_names_.size() == 2) {
    data_ = std::make_shared<ShmManager>(shm_names_[0] + "_data", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names_[0] + "_data_sync", false);
    command_ =
        std::make_shared<ShmManager>(shm_names_[1] + "_command", shm_size_);
    command_sync_ =
        std::make_shared<ShmSync>(shm_names_[1] + "_command_sync", false);
  } else if (shm_names_.size() == 4) {
    data_ = std::make_shared<ShmManager>(shm_names_[0], shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names_[1], false);
    command_ = std::make_shared<ShmManager>(shm_names_[2], shm_size_);
    command_sync_ = std::make_shared<ShmSync>(shm_names_[3], false);
  } else if (shm_names_.size() == 0) {
    data_ = std::make_shared<ShmManager>("mujoco_shm_data", shm_size_);
    command_ = std::make_shared<ShmManager>("mujoco_shm_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>("mujoco_shm_data_sync", false);
    command_sync_ = std::make_shared<ShmSync>("mujoco_shm_command_sync", false);
  } else {
    data_ = std::make_shared<ShmManager>(shm_names_[0] + "_data", shm_size_);
    command_ =
        std::make_shared<ShmManager>(shm_names_[0] + "_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names_[0] + "_data_sync", false);
    command_sync_ =
        std::make_shared<ShmSync>(shm_names_[0] + "_command_sync", false);
  }
  ShmError err = data_->init(false);
  if (err != ShmError::OK) {
    std::cerr << "[ShmClient] Failed to open data shared memory: "
              << static_cast<int>(err) << std::endl;
    return false;
  }

  // 客户端以只读模式打开命令共享内存（用于写入命令）
  err = command_->init(false);
  if (err != ShmError::OK) {
    std::cerr << "[ShmClient] Failed to open command shared memory: "
              << static_cast<int>(err) << std::endl;
    return false;
  }
  return true;
}
ShmClient::~ShmClient() {
  // 客户端不负责清理共享内存
  // 只需要解除映射即可
}

bool ShmClient::WaitForData(int timeout_ms) {
  data_sync_->lock();
  bool wait_result = data_sync_->wait(timeout_ms);
  data_sync_->unlock();
  return wait_result;
}

bool ShmClient::ReceiveAllData(std::vector<JointData> &joint_data,
                               std::vector<SensorData> &sensor_data,
                               std::vector<PoseData> &body_data,
                               std::vector<ActuatorData> &actuator_data) {
  if (!data_ || !data_sync_) {
    std::cerr << "[ShmClient] Data manager or sync not initialized"
              << std::endl;
    return false;
  }

  // 加锁读取数据
  if (!data_sync_->lock()) {
    std::cerr << "[ShmClient] Failed to lock data sync" << std::endl;
    return false;
  }

  // 读取数据大小
  size_t data_size = 0;
  ShmError err = data_->read<size_t>(0, data_size);
  if (err != ShmError::OK || data_size == 0 || data_size > shm_size_) {
    std::cerr << "[ShmClient] Invalid data size: " << data_size << std::endl;
    data_sync_->unlock();
    return false;
  }

  // 读取数据内容
  uint8_t *buf = new uint8_t[data_size];
  err = data_->read_array(sizeof(size_t), buf, data_size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmClient] Failed to read data" << std::endl;
    data_sync_->unlock();
    delete[] buf;
    return false;
  }

  data_sync_->unlock();

  // 解析FlatBuffers数据
  auto data_frame = mujoco_data::GetMujocoDataFrame(buf);
  if (!data_frame) {
    std::cerr << "[ShmClient] Invalid data frame" << std::endl;
    delete[] buf;
    return false;
  }

  // 清空输出容器
  joint_data.clear();
  sensor_data.clear();
  body_data.clear();
  actuator_data.clear();

  // 解析关节数据
  auto joints = data_frame->joints();
  if (joints) {
    for (size_t i = 0; i < joints->size(); i++) {
      auto joint = joints->Get(i);
      JointData jd;
      if (joint->name()) {
        jd.name = joint->name()->str();
      }
      jd.id = joint->joint_id();
      jd.joint_type = static_cast<int>(joint->joint_type());

      // 复制位置数据
      if (joint->qpos()) {
        jd.positions.resize(joint->qpos()->size());
        for (size_t j = 0; j < joint->qpos()->size(); j++) {
          jd.positions[j] = joint->qpos()->Get(j);
        }
      }

      // 复制速度数据
      if (joint->qvel()) {
        jd.velocities.resize(joint->qvel()->size());
        for (size_t j = 0; j < joint->qvel()->size(); j++) {
          jd.velocities[j] = joint->qvel()->Get(j);
        }
      }

      joint_data.push_back(jd);
    }
  }

  // 解析传感器数据
  auto sensors = data_frame->sensors();
  if (sensors) {
    for (size_t i = 0; i < sensors->size(); i++) {
      auto sensor = sensors->Get(i);
      SensorData sd;
      if (sensor->name()) {
        sd.name = sensor->name()->str();
      }
      sd.id = sensor->sensor_id();

      // 复制传感器值
      if (sensor->values()) {
        sd.values.resize(sensor->values()->size());
        for (size_t j = 0; j < sensor->values()->size(); j++) {
          sd.values[j] = sensor->values()->Get(j);
        }
      }

      sensor_data.push_back(sd);
    }
  }

  // 解析身体数据
  auto bodies = data_frame->bodies();
  if (bodies) {
    for (size_t i = 0; i < bodies->size(); i++) {
      auto body = bodies->Get(i);
      PoseData pd;
      if (body->body_name()) {
        pd.name = body->body_name()->str();
      }
      pd.id = body->body_id();

      // 复制位置
      if (body->body_pos() && body->body_pos()->size() == 3) {
        pd.position[0] = body->body_pos()->Get(0);
        pd.position[1] = body->body_pos()->Get(1);
        pd.position[2] = body->body_pos()->Get(2);
      } else {
        pd.position[0] = pd.position[1] = pd.position[2] = 0.0;
      }

      // 复制姿态
      if (body->body_quat() && body->body_quat()->size() == 4) {
        pd.orientation[0] = body->body_quat()->Get(0);
        pd.orientation[1] = body->body_quat()->Get(1);
        pd.orientation[2] = body->body_quat()->Get(2);
        pd.orientation[3] = body->body_quat()->Get(3);
      } else {
        pd.orientation[0] = 1.0;
        pd.orientation[1] = pd.orientation[2] = pd.orientation[3] = 0.0;
      }

      // 复制线速度
      if (body->body_vel() && body->body_vel()->size() == 3) {
        pd.linear_velocity[0] = body->body_vel()->Get(0);
        pd.linear_velocity[1] = body->body_vel()->Get(1);
        pd.linear_velocity[2] = body->body_vel()->Get(2);
      } else {
        pd.linear_velocity[0] = pd.linear_velocity[1] = pd.linear_velocity[2] =
            0.0;
      }

      // 角速度暂时设为0（如果需要可以扩展schema）
      pd.angular_velocity[0] = pd.angular_velocity[1] = pd.angular_velocity[2] =
          0.0;

      body_data.push_back(pd);
    }
    // 解析执行器数据
    auto actuators = data_frame->actuators();
    if (actuators) {
      for (size_t i = 0; i < actuators->size(); i++) {
        auto actuator = actuators->Get(i);
        ActuatorData ad;
        if (actuator->name()) {
          ad.name = actuator->name()->str();
          ad.id = actuator->actuator_id();
          ad.data = actuator->data();
        }
        actuator_data.push_back(ad);
      }
    }
  }

  // 解析执行器数据（如果存在）
  // 注意：当前schema可能没有包含执行器数据
  // 如果需要，可以扩展FlatBuffers schema

  delete[] buf;
  return true;
}

void ShmClient::SendActuatorCommands(
    const std::unordered_map<std::string, double> &actuator_commands) {
  if (!command_ || !command_sync_) {
    std::cerr << "[ShmClient] Command manager or sync not initialized"
              << std::endl;
    return;
  }

  if (actuator_commands.empty()) {
    return;
  }

  // 使用FlatBuffers构建命令帧
  ::flatbuffers::FlatBufferBuilder builder;

  // 只发送第一个命令（根据当前schema）
  // 如果需要发送多个命令，需要修改schema
  auto first_cmd = actuator_commands.begin();
  auto name_offset = builder.CreateString(first_cmd->first);

  mujoco_data::ActuatorCommandBuilder cmd_builder(builder);
  cmd_builder.add_name(name_offset);
  cmd_builder.add_data(first_cmd->second);
  auto cmd = cmd_builder.Finish();

  // 获取当前时间戳（微秒）
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                       now.time_since_epoch())
                       .count();

  mujoco_data::MujocoCommandFrameBuilder frame_builder(builder);
  frame_builder.add_commands(cmd);
  frame_builder.add_timestamp(timestamp);
  auto frame = frame_builder.Finish();

  builder.Finish(frame);

  // 获取序列化数据
  uint8_t *buf = builder.GetBufferPointer();
  size_t size = builder.GetSize();

  if (size > shm_size_) {
    std::cerr << "[ShmClient] Command frame size exceeds shared memory size"
              << std::endl;
    return;
  }

  // 加锁并写入命令
  if (!command_sync_->lock()) {
    std::cerr << "[ShmClient] Failed to lock command sync" << std::endl;
    return;
  }

  ShmError err = command_->write<size_t>(0, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmClient] Failed to write command size" << std::endl;
    command_sync_->unlock();
    return;
  }

  err = command_->write_array(sizeof(size_t), buf, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmClient] Failed to write command data" << std::endl;
    command_sync_->unlock();
    return;
  }

  command_sync_->unlock();
  command_sync_->notify();
}
