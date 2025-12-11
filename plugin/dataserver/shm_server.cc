#include "shm_server.h"
#include "data_frame/command_frame_generated.h"
#include "data_frame/data_frame_generated.h"
#include <cstring>
#include <iostream>

ShmServer::ShmServer(const std::vector<std::string> &shm_names, size_t shm_size)
    : shm_size_(shm_size) {
  // 初始化共享内存管理器和同步机制
  if (shm_names.size() == 1) {
    data_ = std::make_shared<ShmManager>(shm_names[0] + "_data", shm_size_);
    command_ =
        std::make_shared<ShmManager>(shm_names[0] + "_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names[0] + "_data_sync");
    command_sync_ = std::make_shared<ShmSync>(shm_names[0] + "_command_sync");
  } else if (shm_names.size() == 2) {
    data_ = std::make_shared<ShmManager>(shm_names[0] + "_data", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names[0] + "_data_sync");
    command_ =
        std::make_shared<ShmManager>(shm_names[1] + "_command", shm_size_);
    command_sync_ = std::make_shared<ShmSync>(shm_names[1] + "_command_sync");
  } else if (shm_names.size() == 4) {
    data_ = std::make_shared<ShmManager>(shm_names[0], shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names[1]);
    command_ = std::make_shared<ShmManager>(shm_names[2], shm_size_);
    command_sync_ = std::make_shared<ShmSync>(shm_names[3]);
  } else if (shm_names.size() == 0) {
    data_ = std::make_shared<ShmManager>("mujoco_shm_data", shm_size_);
    command_ = std::make_shared<ShmManager>("mujoco_shm_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>("mujoco_shm_data_sync");
    command_sync_ = std::make_shared<ShmSync>("mujoco_shm_command_sync");
  } else {
    data_ = std::make_shared<ShmManager>(shm_names[0] + "_data", shm_size_);
    command_ =
        std::make_shared<ShmManager>(shm_names[0] + "_command", shm_size_);
    data_sync_ = std::make_shared<ShmSync>(shm_names[0] + "_data_sync");
    command_sync_ = std::make_shared<ShmSync>(shm_names[0] + "_command_sync");
  }
  ShmError err = data_->init(true); // 创建数据共享内存
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to create data shared memory: "
              << static_cast<int>(err) << std::endl;
  }
  err = command_->init(true); // 创建命令共享内存
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to create command shared memory: "
              << static_cast<int>(err) << std::endl;
  }
}

ShmServer::~ShmServer() {
  // 先清理共享内存
  // 也可以不清理，让操作系统自动回收
  if (!shm_destroyed_when_done_) {
    return;
  }
  if (data_) {
    data_->destroy();
  }
  if (command_) {
    command_->destroy();
  }
  // 再清理同步对象
  if (data_sync_) {
    data_sync_->destroy();
  }
  if (command_sync_) {
    command_sync_->destroy();
  }
}

void ShmServer::SendJointData(const std::vector<JointData> &joint_data) {
  if (!data_ || !data_sync_) {
    std::cerr << "[ShmServer] Data manager or sync not initialized"
              << std::endl;
    return;
  }

  // 使用FlatBuffers构建数据帧
  ::flatbuffers::FlatBufferBuilder builder;
  std::vector<::flatbuffers::Offset<mujoco_shm::JointData>> joint_offsets;

  for (const auto &joint : joint_data) {
    auto name_offset = builder.CreateString(joint.name);

    mujoco_shm::JointDataBuilder joint_builder(builder);
    joint_builder.add_name(name_offset);
    joint_builder.add_joint_id(joint.id);
    // 使用qpos和qvel存储位置和速度
    if (!joint.positions.empty()) {
      joint_builder.add_qpos(joint.positions[0]);
    }
    if (!joint.velocities.empty()) {
      joint_builder.add_qvel(joint.velocities[0]);
    }
    joint_offsets.push_back(joint_builder.Finish());
  }

  auto joints = builder.CreateVector(joint_offsets);
  mujoco_shm::MujocoDataFrameBuilder data_builder(builder);
  data_builder.add_joints(joints);
  data_builder.add_is_valid(true);
  auto frame = data_builder.Finish();

  builder.Finish(frame);

  // 获取序列化数据
  uint8_t *buf = builder.GetBufferPointer();
  size_t size = builder.GetSize();

  if (size > shm_size_) {
    std::cerr << "[ShmServer] Data frame size exceeds shared memory size"
              << std::endl;
    return;
  }

  // 加锁并写入数据
  if (!data_sync_->lock()) {
    std::cerr << "[ShmServer] Failed to lock data sync" << std::endl;
    return;
  }

  ShmError err = data_->write<size_t>(0, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write data size" << std::endl;
    data_sync_->unlock();
    return;
  }

  err = data_->write_array(sizeof(size_t), buf, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write joint data" << std::endl;
    data_sync_->unlock();
    return;
  }

  data_sync_->unlock();
  data_sync_->notify();
}

void ShmServer::SendSensorData(const std::vector<SensorData> &sensor_data) {
  if (!data_ || !data_sync_) {
    std::cerr << "[ShmServer] Data manager or sync not initialized"
              << std::endl;
    return;
  }

  // 使用FlatBuffers构建传感器数据帧
  ::flatbuffers::FlatBufferBuilder builder;
  std::vector<::flatbuffers::Offset<mujoco_shm::SensorData>> sensor_offsets;

  for (const auto &sensor : sensor_data) {
    auto name_offset = builder.CreateString(sensor.name);
    auto values_offset = builder.CreateVector(sensor.values);

    mujoco_shm::SensorDataBuilder sensor_builder(builder);
    sensor_builder.add_name(name_offset);
    sensor_builder.add_sensor_id(sensor.id);
    sensor_builder.add_values(values_offset);
    sensor_offsets.push_back(sensor_builder.Finish());
  }

  auto sensors = builder.CreateVector(sensor_offsets);
  mujoco_shm::MujocoDataFrameBuilder data_builder(builder);
  data_builder.add_sensors(sensors);
  data_builder.add_is_valid(true);
  auto frame = data_builder.Finish();

  builder.Finish(frame);

  // 获取序列化数据
  uint8_t *buf = builder.GetBufferPointer();
  size_t size = builder.GetSize();

  if (size > shm_size_) {
    std::cerr << "[ShmServer] Sensor data frame size exceeds shared memory size"
              << std::endl;
    return;
  }

  // 加锁并写入数据
  if (!data_sync_->lock()) {
    std::cerr << "[ShmServer] Failed to lock data sync" << std::endl;
    return;
  }

  ShmError err = data_->write<size_t>(0, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write sensor data size" << std::endl;
    data_sync_->unlock();
    return;
  }

  err = data_->write_array(sizeof(size_t), buf, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write sensor data" << std::endl;
    data_sync_->unlock();
    return;
  }

  data_sync_->unlock();
  data_sync_->notify();
}

void ShmServer::SendBodyData(const std::vector<PoseData> &body_data) {
  if (!data_ || !data_sync_) {
    std::cerr << "[ShmServer] Data manager or sync not initialized"
              << std::endl;
    return;
  }

  // 使用FlatBuffers构建身体数据帧
  ::flatbuffers::FlatBufferBuilder builder;
  std::vector<::flatbuffers::Offset<mujoco_shm::PoseData>> pose_offsets;

  for (const auto &pose : body_data) {
    auto name_offset = builder.CreateString(pose.name);

    std::vector<double> position(pose.position, pose.position + 3);
    std::vector<double> orientation(pose.orientation, pose.orientation + 4);
    std::vector<double> linear_vel(pose.linear_velocity,
                                   pose.linear_velocity + 3);
    std::vector<double> angular_vel(pose.angular_velocity,
                                    pose.angular_velocity + 3);

    auto pos_offset = builder.CreateVector(position);
    auto orient_offset = builder.CreateVector(orientation);
    auto lin_vel_offset = builder.CreateVector(linear_vel);

    mujoco_shm::PoseDataBuilder pose_builder(builder);
    pose_builder.add_body_name(name_offset);
    pose_builder.add_body_id(pose.id);
    pose_builder.add_body_pos(pos_offset);
    pose_builder.add_body_quat(orient_offset);
    pose_builder.add_body_vel(lin_vel_offset);
    pose_offsets.push_back(pose_builder.Finish());
  }

  auto poses = builder.CreateVector(pose_offsets);
  mujoco_shm::MujocoDataFrameBuilder data_builder(builder);
  data_builder.add_bodies(poses);
  data_builder.add_is_valid(true);
  auto frame = data_builder.Finish();

  builder.Finish(frame);

  // 获取序列化数据
  uint8_t *buf = builder.GetBufferPointer();
  size_t size = builder.GetSize();

  if (size > shm_size_) {
    std::cerr << "[ShmServer] Body data frame size exceeds shared memory size"
              << std::endl;
    return;
  }

  // 加锁并写入数据
  if (!data_sync_->lock()) {
    std::cerr << "[ShmServer] Failed to lock data sync" << std::endl;
    return;
  }

  ShmError err = data_->write<size_t>(0, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write body data size" << std::endl;
    data_sync_->unlock();
    return;
  }

  err = data_->write_array(sizeof(size_t), buf, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to write body data" << std::endl;
    data_sync_->unlock();
    return;
  }

  data_sync_->unlock();
  data_sync_->notify();
}

void ShmServer::ReceiveActuatorCommands(
    std::unordered_map<std::string, double> &actuator_commands) {
  if (!command_ || !command_sync_) {
    std::cerr << "[ShmServer] Command manager or sync not initialized"
              << std::endl;
    return;
  }

  // 尝试等待命令数据（非阻塞）
  if (!command_sync_->lock()) {
    std::cerr << "[ShmServer] Failed to lock command sync" << std::endl;
    return;
  }

  // 读取数据大小
  size_t size = 0;
  ShmError err = command_->read<size_t>(0, size);
  if (err != ShmError::OK || size == 0 || size > shm_size_) {
    command_sync_->unlock();
    return;
  }

  // 读取FlatBuffers数据
  uint8_t *buf = new uint8_t[size];
  err = command_->read_array(sizeof(size_t), buf, size);
  if (err != ShmError::OK) {
    std::cerr << "[ShmServer] Failed to read command data" << std::endl;
    command_sync_->unlock();
    delete[] buf;
    return;
  }

  command_sync_->unlock();

  // 解析FlatBuffers数据
  auto command_frame = mujoco_data::GetMujocoCommandFrame(buf);
  if (!command_frame) {
    std::cerr << "[ShmServer] Invalid command frame data" << std::endl;
    delete[] buf;
    return;
  }

  // 提取执行器命令
  auto commands = command_frame->commands();
  if (commands) {
    for (size_t i = 0; i < commands->size(); ++i) {
      auto cmd = commands->Get(i);
      if (cmd->name() && cmd->pos()) {
        std::string actuator_name = cmd->name()->str();
        if (cmd->pos()->size() > 0) {
          actuator_commands[actuator_name] = cmd->pos()->Get(0);
        }
      }
    }
  }

  delete[] buf;
}