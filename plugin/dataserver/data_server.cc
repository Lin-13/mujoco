// data_server.cpp
#include "data_server.h"
#include "shm_server.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <sstream>
#include <vector>
using namespace std::chrono;
using namespace std::chrono_literals;
namespace mujoco::plugin::dataserver {
std::unordered_map<int, void *> DataServer::INSTANCE_LIST;
DataServer::DataServer(std::string server_args_str, int instance)
    : instance_(instance) {
  if (!server_args_str.empty()) {
    std::istringstream iss(server_args_str);
    std::string arg;
    while (std::getline(iss, arg, ';')) {
      // 去除空格
      arg.erase(0, arg.find_first_not_of(' '));
      arg.erase(arg.find_last_not_of(' ') + 1);
      if (!arg.empty()) {
        server_args_.push_back(arg);
      }
    }
  }
  if (!IS_SINGLETON) {
    for (auto &arg : server_args_) {
      arg = arg + "_" + std::to_string(this->instance_);
      std::cout << "[DataServer] Server arg: " << arg << std::endl;
    }
  }
  INSTANCE_LIST[instance_] = this;
}
DataServer::~DataServer() {
  std::cout << "[DataServer] Instance " << instance_ << " destroyed"
            << std::endl;
  INSTANCE_LIST.erase(instance_);
  if (server_thread_.joinable()) {
    stop_thread_ = true;
    server_thread_.join();
  }
}
void DataServer::InitializeJointDataBuffer(const mjModel *m,
                                           const char *joints_config) {
  if (!joints_config || strlen(joints_config) == 0) {
    std::cout << "[DataServer] No joints specified for monitoring" << std::endl;
    return;
  }

  // 解析关节配置字符串（格式："joint1;joint2;joint3" 或 "all"）
  std::string config(joints_config);
  std::istringstream iss(config);
  std::string joint_name;
  joint_data_.clear();
  joint_data_.reserve(m->njnt);
  if (config == "all") {
    // 监控所有关节
    for (int joint_id = 0; joint_id < m->njnt; joint_id++) {
      const char *name = mj_id2name(m, mjOBJ_JOINT, joint_id);
      if (name) {
        joint_ids_.push_back(joint_id);
        joint_names_.push_back(name);
        int qpos_adr = m->jnt_qposadr[joint_id];
        int nqpos = 0;
        if (qpos_adr >= 0) {
          // 计算这个关节有多少位置坐标
          if (joint_id + 1 < m->njnt) {
            nqpos = m->jnt_qposadr[joint_id + 1] - qpos_adr;
          } else {
            nqpos = m->nq - qpos_adr;
          }
        }
        int dof_adr = m->jnt_dofadr[joint_id];
        int ndof = 0;
        if (dof_adr >= 0) {
          // 计算这个关节有多少自由度
          if (joint_id + 1 < m->njnt) {
            ndof = m->jnt_dofadr[joint_id + 1] - dof_adr;
          } else {
            ndof = m->nv - dof_adr;
          }
        }
        joint_data_.emplace_back(
            JointData({name, joint_id, std::vector<mjtNum>(nqpos, 0),
                       std::vector<mjtNum>(ndof, 0), m->jnt_type[joint_id]}));
        std::cout << "[DataServer] Monitoring joint: " << name
                  << " (ID: " << joint_id
                  << ", qpos_adr: " << m->jnt_qposadr[joint_id] << ")"
                  << std::endl;
      }
    }
  } else {
    // 监控指定关节
    while (std::getline(iss, joint_name, ';')) {
      // 去除空格
      joint_name.erase(0, joint_name.find_first_not_of(' '));
      joint_name.erase(joint_name.find_last_not_of(' ') + 1);

      if (!joint_name.empty()) {
        int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (joint_id >= 0) {
          joint_ids_.push_back(joint_id);
          joint_names_.push_back(joint_name);
          int qpos_adr = m->jnt_qposadr[joint_id];
          int nqpos = 0;
          if (qpos_adr >= 0) {
            if (joint_id + 1 < m->njnt) {
              nqpos = m->jnt_qposadr[joint_id + 1] - qpos_adr;
            } else {
              nqpos = m->nq - qpos_adr;
            }
          }
          int dof_adr = m->jnt_dofadr[joint_id];
          int ndof = 0;
          if (dof_adr >= 0) {
            if (joint_id + 1 < m->njnt) {
              ndof = m->jnt_dofadr[joint_id + 1] - dof_adr;
            } else {
              ndof = m->nv - dof_adr;
            }
          }
          joint_data_.emplace_back(
              JointData({joint_name, joint_id, std::vector<mjtNum>(nqpos, 0),
                         std::vector<mjtNum>(ndof, 0), m->jnt_type[joint_id]}));
          std::cout << "[DataServer] Monitoring joint: " << joint_name
                    << " (ID: " << joint_id
                    << ", qpos_adr: " << m->jnt_qposadr[joint_id] << ")"
                    << std::endl;
        } else {
          std::cerr << "[DataServer] Warning: Joint '" << joint_name
                    << "' not found" << std::endl;
        }
      }
    }
  }
}

void DataServer::InitializeBodyDataBuffer(const mjModel *m,
                                          const char *bodies_config) {
  if (!bodies_config || strlen(bodies_config) == 0) {
    std::cout << "[DataServer] No bodies specified for monitoring" << std::endl;
    return;
  }

  // 解析身体配置字符串（格式："body1;body2;body3" 或 "all"）
  std::string config(bodies_config);
  std::istringstream iss(config);
  std::string body_name;
  body_data_.clear();
  body_data_.reserve(m->nbody);
  if (config == "all") {
    // 监控所有身体
    for (int i = 0; i < m->nbody; i++) {
      const char *name = mj_id2name(m, mjOBJ_BODY, i);
      if (name) {
        body_ids_.push_back(i);
        body_names_.push_back(name);
        body_xpos_adr_.push_back(3 * i);      // xpos 是 3个连续值 (x, y, z)
        body_xquat_adr_.push_back(4 * i);     // xquat 是 4个连续值 (w, x, y, z)
        body_xvelp_adr_.push_back(6 * i);     // cvel 中前3个是线速度
        body_xvelr_adr_.push_back(6 * i + 3); // cvel 中后3个是角速度
        body_data_.emplace_back(
            PoseData{name, i, {0, 0, 0}, {1, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
        std::cout << "[DataServer] Monitoring body: " << name << " (ID: " << i
                  << ", xpos_adr: " << (3 * i) << ", xquat_adr: " << (4 * i)
                  << ", cvel_adr: " << (6 * i) << ")" << std::endl;
      }
    }
  } else {
    // 监控指定身体
    while (std::getline(iss, body_name, ';')) {
      // 去除空格
      body_name.erase(0, body_name.find_first_not_of(' '));
      body_name.erase(body_name.find_last_not_of(' ') + 1);

      if (!body_name.empty()) {
        int body_id = mj_name2id(m, mjOBJ_BODY, body_name.c_str());
        if (body_id >= 0) {
          body_ids_.push_back(body_id);
          body_names_.push_back(body_name);
          body_xpos_adr_.push_back(3 * body_id);
          body_xquat_adr_.push_back(4 * body_id);
          body_xvelp_adr_.push_back(6 * body_id);
          body_xvelr_adr_.push_back(6 * body_id + 3);
          body_data_.emplace_back(PoseData{body_name,
                                           body_id,
                                           {0, 0, 0},
                                           {1, 0, 0, 0},
                                           {0, 0, 0},
                                           {0, 0, 0}});
          std::cout << "[DataServer] Monitoring body: " << body_name
                    << " (ID: " << body_id << ", xpos_adr: " << (3 * body_id)
                    << ", xquat_adr: " << (4 * body_id)
                    << ", cvel_adr: " << (6 * body_id) << ")" << std::endl;
        } else {
          std::cerr << "[DataServer] Warning: Body '" << body_name
                    << "' not found" << std::endl;
        }
      }
    }
  }
}
void DataServer::GetCameraSensorData(const mjModel *m, const mjData *d) {}

void DataServer::GetJointData(const mjModel *m, const mjData *d) {
  for (auto &joint : joint_data_) {
    int qpos_adr = m->jnt_qposadr[joint.id];
    int dof_adr = m->jnt_dofadr[joint.id];

    // 获取位置数据
    for (size_t i = 0; i < joint.positions.size(); i++) {
      if (qpos_adr + static_cast<int>(i) < m->nq) {
        joint.positions[i] = d->qpos[qpos_adr + i];
      } else {
        joint.positions[i] = 0.0;
      }
    }

    // 获取速度数据
    for (size_t i = 0; i < joint.velocities.size(); i++) {
      if (dof_adr + static_cast<int>(i) < m->nv) {
        joint.velocities[i] = d->qvel[dof_adr + i];
      } else {
        joint.velocities[i] = 0.0;
      }
    }
  }
}
void DataServer::GetActuatorData(const mjModel *m, const mjData *d) {
  for (auto &actuator : actuator_data_) {
    int actuator_id = actuator.id;
    if (actuator_id >= 0 && actuator_id < m->nu) {
      actuator.data = d->actuator_force[actuator_id];
    } else {
      actuator.data = 0.0;
    }
  }
}
void DataServer::GetBodyPoseData(const mjModel *m, const mjData *d) {
  // size_t count_body = static_cast<int>(body_ids_.size());
  // body_positions_.clear();
  // body_orientations_.clear();
  // body_velocities_.clear();
  // body_positions_.reserve(count_body * 3);
  // body_orientations_.reserve(count_body * 4);
  // body_velocities_.reserve(count_body * 6);
  // for (size_t i = 0; i < count_body; i++) {
  //   int body_id = body_ids_[i];

  //   // 检查ID是否有效
  //   if (body_id < 0 || body_id >= m->nbody) {
  //     continue;
  //   }

  //   // 获取位置 (x, y, z)
  //   int pos_adr = body_xpos_adr_[i];
  //   if (pos_adr >= 0 && pos_adr + 2 < 3 * m->nbody) {
  //     body_positions_.push_back(d->xpos[pos_adr]);     // x
  //     body_positions_.push_back(d->xpos[pos_adr + 1]); // y
  //     body_positions_.push_back(d->xpos[pos_adr + 2]); // z
  //   } else {
  //     body_positions_.push_back(0.0);
  //     body_positions_.push_back(0.0);
  //     body_positions_.push_back(0.0);
  //   }

  //   // 获取姿态四元数 (w, x, y, z)
  //   int quat_adr = body_xquat_adr_[i];
  //   if (quat_adr >= 0 && quat_adr + 3 < 4 * m->nbody) {
  //     body_orientations_.push_back(d->xquat[quat_adr]);     // w
  //     body_orientations_.push_back(d->xquat[quat_adr + 1]); // x
  //     body_orientations_.push_back(d->xquat[quat_adr + 2]); // y
  //     body_orientations_.push_back(d->xquat[quat_adr + 3]); // z
  //   } else {
  //     body_orientations_.push_back(1.0); // w
  //     body_orientations_.push_back(0.0); // x
  //     body_orientations_.push_back(0.0); // y
  //     body_orientations_.push_back(0.0); // z
  //   }

  //   // 获取速度 (线速度 + 角速度)
  //   int velp_adr = body_xvelp_adr_[i];
  //   int velr_adr = body_xvelr_adr_[i];
  //   if (velp_adr >= 0 && velp_adr + 2 < 6 * m->nbody && velr_adr >= 0 &&
  //       velr_adr + 2 < 6 * m->nbody) {
  //     // 线速度 (vx, vy, vz)
  //     body_velocities_.push_back(d->cvel[velp_adr]);     // vx
  //     body_velocities_.push_back(d->cvel[velp_adr + 1]); // vy
  //     body_velocities_.push_back(d->cvel[velp_adr + 2]); // vz

  //     // 角速度 (wx, wy, wz)
  //     body_velocities_.push_back(d->cvel[velr_adr]);     // wx
  //     body_velocities_.push_back(d->cvel[velr_adr + 1]); // wy
  //     body_velocities_.push_back(d->cvel[velr_adr + 2]); // wz
  //   } else {
  //     body_velocities_.push_back(0.0); // vx
  //     body_velocities_.push_back(0.0); // vy
  //     body_velocities_.push_back(0.0); // vz
  //     body_velocities_.push_back(0.0); // wx
  //     body_velocities_.push_back(0.0); // wy
  //     body_velocities_.push_back(0.0); // wz
  //   }
  // }
  // body_data_
  for (auto &body : body_data_) {
    int body_id = body.id;
    // 位置
    int pos_adr = 3 * body_id;
    body.position[0] = d->xpos[pos_adr];
    body.position[1] = d->xpos[pos_adr + 1];
    body.position[2] = d->xpos[pos_adr + 2];
    // 姿态
    int quat_adr = 4 * body_id;
    body.orientation[0] = d->xquat[quat_adr];
    body.orientation[1] = d->xquat[quat_adr + 1];
    body.orientation[2] = d->xquat[quat_adr + 2];
    body.orientation[3] = d->xquat[quat_adr + 3];
    // 线速度和角速度
    int velp_adr = 6 * body_id;
    int velr_adr = 6 * body_id + 3;
    body.linear_velocity[0] = d->cvel[velp_adr];
    body.linear_velocity[1] = d->cvel[velp_adr + 1];
    body.linear_velocity[2] = d->cvel[velp_adr + 2];
    body.angular_velocity[0] = d->cvel[velr_adr];
    body.angular_velocity[1] = d->cvel[velr_adr + 1];
    body.angular_velocity[2] = d->cvel[velr_adr + 2];
  }
}
std::vector<double> DataServer::GetJointPositions() {
  std::vector<double> positions;
  for (const auto &joint : joint_data_) {
    positions.insert(positions.end(), joint.positions.begin(),
                     joint.positions.end());
  }
  return positions;
}
int DataServer::GetNumBodyData() const {
  // 每个身体提供：3个位置 + 4个姿态 + 6个速度 = 13个数据
  return static_cast<int>(body_ids_.size() * 13);
}
int DataServer::GetNumSensorData() const {
  return static_cast<int>(sensor_data_.size());
}
void DataServer::InitializeSensorDataBuffer(const mjModel *m,
                                            const char *sensors_config) {
  if (!sensors_config || strlen(sensors_config) == 0) {
    std::cout << "[DataServer] No sensors specified for monitoring"
              << std::endl;
    return;
  }

  // 解析传感器配置字符串（格式："sensor1;sensor2;sensor3" 或 "all"）
  std::string config(sensors_config);
  std::istringstream iss(config);
  std::string sensor_name;

  if (config == "all") {
    // 监控所有传感器
    for (int i = 0; i < m->nsensor; i++) {
      const char *name = mj_id2name(m, mjOBJ_SENSOR, i);
      std::cout << "[DataServer] Checking sensor: " << name << " (ID: " << i
                << ")" << std::endl;
      // 判断是否DataServer的插件传感器
      bool is_dataserver_plugin_sensor = false;
      bool is_empty_dataserver = false;
      if (m->sensor_type[i] == mjSENS_PLUGIN) {
        int instance = m->sensor_plugin[i];
        for (auto &[instance_in_list, pointer] : INSTANCE_LIST) {
          if (instance_in_list == instance) {
            is_dataserver_plugin_sensor = true;
            if (pointer == nullptr) {
              is_empty_dataserver = true;
              std::cout << "[DataServer] Found DataServer plugin sensor with "
                           "null pointer: "
                        << name << " (ID: " << i << ")" << std::endl;
            }
            break;
          }
        }
      }
      is_dataserver_plugin_sensor = false;
      // ? 是否需要排除DataServer自己的传感器
      if (name && !is_dataserver_plugin_sensor && !is_empty_dataserver) {
        int sensor_dim = m->sensor_dim[i];
        sensor_data_.emplace_back(
            SensorData{name, i, std::vector<mjtNum>(sensor_dim, 0)});
        std::cout << "[DataServer] Monitoring sensor: " << name << " (ID: " << i
                  << ")" << std::endl;
      }
    }
    std::cout << "[DataServer] Total sensors monitored: " << sensor_data_.size()
              << std::endl;
  } else {
    // 监控指定传感器
    while (std::getline(iss, sensor_name, ';')) {
      // 去除空格
      sensor_name.erase(0, sensor_name.find_first_not_of(' '));
      sensor_name.erase(sensor_name.find_last_not_of(' ') + 1);

      if (!sensor_name.empty()) {
        int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name.c_str());
        if (sensor_id >= 0) {
          int sensor_dim = m->sensor_dim[sensor_id];
          sensor_data_.emplace_back(SensorData{
              sensor_name, sensor_id, std::vector<mjtNum>(sensor_dim, 0)});
          std::cout << "[DataServer] Monitoring sensor: " << sensor_name
                    << " (ID: " << sensor_id << ")" << std::endl;
        } else {
          std::cerr << "[DataServer] Warning: Sensor '" << sensor_name
                    << "' not found" << std::endl;
        }
      }
    }
  }
}
void DataServer::GetSensorData(const mjModel *m, const mjData *d) {
  for (auto &sensor : sensor_data_) {
    int sensor_id = sensor.id;
    size_t sensor_dim = sensor.values.size();
    int adr = m->sensor_adr[sensor_id];
    for (int i = 0; i < sensor_dim; i++) {
      sensor.values[i] = d->sensordata[adr + i];
    }
  }
}
void DataServer::InitializeJointActuators(const mjModel *m,
                                          const char *actuators_config) {
  if (!actuators_config || strlen(actuators_config) == 0) {
    std::cout << "[DataServer] No actuators specified for control" << std::endl;
    return;
  }

  // 解析执行器配置字符串（格式："actuator1;actuator2" 或 "all"）
  std::string config(actuators_config);
  std::istringstream iss(config);
  std::string actuator_spec;

  if (config == "all") {
    for (int i = 0; i < m->nu; i++) {
      const char *name = mj_id2name(m, mjOBJ_ACTUATOR, i);
      if (name) {
        actuator_ids_.push_back(i);
        actuator_names_.push_back(name);
        actuator_data_.emplace_back(ActuatorData{name, i, 0});
        std::cout << "[DataServer] Controlling actuator: " << name
                  << " (ID: " << i << ")" << std::endl;
      }
    }
  } else {
    // 监控指定执行器
    while (std::getline(iss, actuator_spec, ';')) {
      // 去除空格
      actuator_spec.erase(0, actuator_spec.find_first_not_of(' '));
      actuator_spec.erase(actuator_spec.find_last_not_of(' ') + 1);

      if (actuator_spec.empty()) {
        continue;
      }
      std::string actuator_name;
      actuator_name = actuator_spec;
      int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name.c_str());
      if (actuator_id >= 0) {
        actuator_ids_.push_back(actuator_id);
        actuator_names_.push_back(actuator_name);
        actuator_data_.emplace_back(
            ActuatorData{actuator_name, actuator_id, 0});
        std::cout << "[DataServer] Controlling actuator: " << actuator_name
                  << " (ID: " << actuator_id << ")" << std::endl;
      } else {
        std::cerr << "[DataServer] Warning: Actuator '" << actuator_name
                  << "' not found" << std::endl;
      }
    }
  }
}

std::unique_ptr<DataServer> DataServer::Create(const mjModel *m, int instance) {
  if (IS_SINGLETON) {
    for (auto &[instance_in_list, pointer] : INSTANCE_LIST) {
      if (pointer != nullptr) {
        std::cout << "[DataServer] Warning: Singleton instance already exists: "
                  << instance_in_list << ", set current instance " << instance
                  << " to nullptr." << std::endl;
        INSTANCE_LIST[instance] = nullptr;
        return nullptr;
      }
    }
  }
  std::cout << "[DataServer] Creating instance " << instance << std::endl;
  // 获取mjModel的结构
  // 读取端口配置
  // instance_name_ = mj_getPluginInstanceName(m, instance);
  // 创建服务器实例
  const char *server_args_str = mj_getPluginConfig(m, instance, "server_args");
  auto server = std::make_unique<DataServer>(
      server_args_str ? std::string(server_args_str) : "", instance);
  server->model_tree_root_ = ModelTreeBuilder::buildTreeFromModel(m);
  server->model_mjcf_ = ModelTreeBuilder::generateMJCF(m);
  // std::cout << "[DataServer] Model MJCF:\n" << server->model_mjcf_ <<
  // std::endl;

  // 读取关节配置
  const char *joints_config = mj_getPluginConfig(m, instance, "joints");
  server->InitializeJointDataBuffer(m, joints_config);

  // 读取执行器配置
  const char *actuators_config = mj_getPluginConfig(m, instance, "actuators");
  server->InitializeJointActuators(m, actuators_config);
  // 读取身体配置
  const char *bodies_config = mj_getPluginConfig(m, instance, "bodies");
  server->InitializeBodyDataBuffer(m, bodies_config);
  // 读取传感器配置
  const char *sensors_config = mj_getPluginConfig(m, instance, "sensors");
  server->InitializeSensorDataBuffer(m, sensors_config);
  // 启动服务器
  const char *async_config = mj_getPluginConfig(m, instance, "async");
  if (async_config && (std::string(async_config) == "true" ||
                       std::string(async_config) == "1")) {
    server->is_server_async_ = true;
    server->StartServer();
  } else {
    server->is_server_async_ = false;
  }
  // server->StartServer();

  return server;
}
void DataServer::Destroy(mjData *d, int instance) {
  std::cout << "[DataServer] Destroy called for instance " << instance
            << std::endl;
  // delete
  if (d->plugin_data[instance] == 0) {
    std::cout << "[DataServer] Warning: No plugin data to destroy for instance "
              << instance << std::endl;
    return;
  }
  delete reinterpret_cast<DataServer *>(d->plugin_data[instance]);
}
void DataServer::Reset(mjtNum *plugin_state) {
  std::cout << "[DataServer] Reset called" << std::endl;
  // 重置状态
  is_initialized_ = false;
}

void DataServer::Compute(const mjModel *m, mjData *d, int instance) {
  // static int count = 0;
  // count++;
  // static steady_clock::time_point last_time = steady_clock::now();

  // 更新当前仿真时间
  current_sim_time_.store(d->time);

  // 获取数据并写入到缓冲区
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    GetJointData(m, d);
    GetBodyPoseData(m, d);
    GetSensorData(m, d);
    GetActuatorData(m, d);
  }
  if (is_server_async_ == false) {
    {
      // 发送传感器数据（包括关节和身体数据）
      std::unique_lock<std::mutex> lock(data_mutex_);
      SendData();
    }
    {
      std::unique_lock<std::mutex> lock(command_mutex_);
      // 接收控制命令
      ReceiveControlCommands();
    }
  }

  // 每100步输出一次信息
  // if (count % 100 == 0) {
  // auto now = steady_clock::now();
  // auto duration = duration_cast<milliseconds>(now - last_time).count();
  // last_time = now;
  // std::cout << "[DataServer] Compute step duration: " << duration << " ms"
  //           << std::endl;
  // std::cout << "[DataServer] Compute called " << count << " times"
  //           << std::endl;

  // 输出关节角度信息
  // for (size_t i = 0; i < joint_ids_.size(); i++) {
  //   std::cout << "[DataServer] Joint: " << joint_names_[i]
  //             << " (ID: " << joint_ids_[i] << ")"
  //             << " Position: " << joint_data_[i].positions[0]
  //             << " Velocity: " << joint_data_[i].velocities[0] << std::endl;
  // }
  // 打印每一个body的数据（用于调试）
  // for (size_t i = 0; i < body_ids_.size(); i++) {
  //   std::cout << "[DataServer] Body: " << body_names_[i]
  //             << " (ID: " << body_ids_[i] << ")\n";
  //   std::cout << "  Position: (" << body_positions_[i * 3] << ", "
  //             << body_positions_[i * 3 + 1] << ", "
  //             << body_positions_[i * 3 + 2] << ")\n";
  //   std::cout << "  Orientation (quat): (" << body_orientations_[i * 4]
  //             << ", " << body_orientations_[i * 4 + 1] << ", "
  //             << body_orientations_[i * 4 + 2] << ", "
  //             << body_orientations_[i * 4 + 3] << ")\n";
  //   std::cout << "  Velocity: (" << body_velocities_[i * 6] << ", "
  //             << body_velocities_[i * 6 + 1] << ", "
  //             << body_velocities_[i * 6 + 2] << ", "
  //             << body_velocities_[i * 6 + 3] << ", "
  //             << body_velocities_[i * 6 + 4] << ", "
  //             << body_velocities_[i * 6 + 5] << ")\n";
  // }
  // for (const auto &body : body_data_) {
  //   std::cout << "[DataServer] Body: " << body.name << " (ID: " << body.id
  //             << ")\n";
  //   std::cout << "  Position: (" << body.position[0] << ", "
  //             << body.position[1] << ", " << body.position[2] << ")\n";
  //   std::cout << "  Orientation (quat): (" << body.orientation[0] << ", "
  //             << body.orientation[1] << ", " << body.orientation[2] << ", "
  //             << body.orientation[3] << ")\n";
  //   std::cout << "  Linear Velocity: (" << body.linear_velocity[0] << ", "
  //             << body.linear_velocity[1] << ", " << body.linear_velocity[2]
  //             << ")\n";
  //   std::cout << "  Angular Velocity: (" << body.angular_velocity[0] << ", "
  //             << body.angular_velocity[1] << ", " << body.angular_velocity[2]
  //             << ")\n";
  // }
  // 输出传感器数据
  // for (const auto &sensor : sensor_data_) {
  //   std::cout << "[DataServer] Sensor: " << sensor.name
  //             << " (ID: " << sensor.id << ") Values: ";
  //   for (const auto &value : sensor.values) {
  //     std::cout << value << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // }
}
// 在另一个线程启动服务器，通过缓冲区共享的方式传输数据，避免数据传输不稳定对仿真循环的影响
// 同时过滤command
void DataServer::StartServer() {
  if (server_args_.empty()) {
    server_args_.push_back("mujoco_data_" + std::to_string(instance_));
    server_args_.push_back("mujoco_command_" + std::to_string(instance_));
  }
  // 计算缓冲区大小
  size_t default_shm_size = 4 * 1024 * 1024;
  server_ = std::make_shared<ShmServer>(server_args_, default_shm_size);
  server_thread_ = std::thread([this]() {
    // 临时缓冲区
    MujocoDataFrame temp_data_frame{.joints{this->joint_data_},
                                    .sensors{this->sensor_data_},
                                    .bodies{this->body_data_},
                                    .actuators{this->actuator_data_}};
    MujocoCommandFrame temp_actuator_commands;
    // TODO: 经过测试无法达到update_rate_，需要进一步优化
    auto start_time = std::chrono::steady_clock::now();
    auto period =
        std::chrono::microseconds(static_cast<int64_t>(1.0e6 / update_rate_));
    while (!this->stop_thread_) {
      // this->server_->Update();
      auto next_tick = start_time + period;
      start_time = next_tick;
      std::this_thread::sleep_until(next_tick - std::chrono::microseconds(200));
      while (1) {
        if (std::chrono::steady_clock::now() >= next_tick) {
          break;
        }
      }
      {
        // 获取数据并写入到缓冲区
        std::unique_lock<std::mutex> lock(this->data_mutex_);
        temp_data_frame.bodies = this->body_data_;
        temp_data_frame.joints = this->joint_data_;
        temp_data_frame.sensors = this->sensor_data_;
        temp_data_frame.actuators = this->actuator_data_;
      }
      // header
      temp_data_frame.timestamp =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
      temp_data_frame.frame_id = ++data_frame_id_;
      temp_data_frame.req_frame_id = current_req_frame_id_;
      temp_data_frame.is_valid = true;
      temp_data_frame.desctrption = "MuJoCo Data Server";
      // 使用真实的仿真时间
      temp_data_frame.sim_time = current_sim_time_.load();
      this->server_->SendAllData(temp_data_frame);
      this->server_->ReceiveActuatorCommands(temp_actuator_commands);
      {
        // 更新控制命令缓冲区
        std::unique_lock<std::mutex> lock(this->command_mutex_);
        current_req_frame_id_ =
            temp_actuator_commands.frame_id; // 记录请求frame_id
        actuator_commands_.clear();
        for (size_t i = 0; i < actuator_ids_.size(); i++) {
          const std::string &actuator_name = actuator_names_[i];
          if (temp_actuator_commands.commands.find(actuator_name) !=
              temp_actuator_commands.commands.end()) {
            actuator_commands_.emplace(
                actuator_name, temp_actuator_commands.commands[actuator_name]);
          }
        }
      }
    }
  });
}
void DataServer::SendData() {
  if (server_) {
    std::unique_lock<std::mutex> lock(this->data_mutex_);
    MujocoDataFrame temp_frame;
    temp_frame.joints = this->joint_data_;
    temp_frame.sensors = this->sensor_data_;
    temp_frame.bodies = this->body_data_;
    temp_frame.actuators = this->actuator_data_;
    temp_frame.timestamp =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    temp_frame.frame_id = ++data_frame_id_;
    temp_frame.req_frame_id = current_req_frame_id_;
    temp_frame.is_valid = true;
    temp_frame.desctrption = "MuJoCo Data Server";
    temp_frame.sim_time = current_sim_time_.load();
    this->server_->SendAllData(temp_frame);
  }
}
// 写入actuator_names_
void DataServer::ReceiveControlCommands() {
  if (server_) {
    MujocoCommandFrame actuator_commands;
    this->server_->ReceiveActuatorCommands(actuator_commands);
    std::unique_lock<std::mutex> lock(this->command_mutex_);
    current_req_frame_id_ = actuator_commands.frame_id; // 记录请求frame_id
    actuator_commands_ = actuator_commands.commands;
  }
}
// 将缓冲区的控制命令应用到mjData:读取actuator_commands_
void DataServer::UpdateActuatorControls(const mjModel *m, mjData *d) {
  for (size_t i = 0; i < actuator_data_.size(); i++) {
    int actuator_id = actuator_data_[i].id;
    const std::string &actuator_name = actuator_data_[i].name;
    if (actuator_commands_.count(actuator_name) == 0) {
      continue;
    }
    if (actuator_id < m->nu) {
      double received_command = actuator_commands_[actuator_name];
      // 应用增益并设置控制信号
      d->ctrl[actuator_id] = received_command;
    }
  }
}
void DataServer::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.dataserver";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;

  // 定义插件属性
  std::vector<const char *> attributes = {
      "server_args", // 传入服务器的参数，格式："arg1;arg2;arg3"
      "joints",      // 要监控的关节（分号分隔，或"all"）
      "actuators",   // 要控制的执行器（格式："name;name2"）
      "bodies",      // 要监控的身体（分号分隔，或"all"）
      "sensors",     // 要监控的传感器（分号分隔，或"all"）
      "async",       // 是否异步运行服务器（true/false）
  };

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  // 状态数量（如果需要保持状态）
  plugin.nstate = +[](const mjModel *m, int instance) -> int {
    return 0; // 不需要状态
  };

  // 传感器数据数量（根据监控的关节数量动态计算）
  plugin.nsensordata =
      +[](const mjModel *m, int instance, int sensor_id) -> int {
    // 读取关节配置
    // 当为空实例的时候直接返回0
    // DataServer *data_server = nullptr;
    // for (auto &[pointer, instance_in_list] : INSTANCE_LIST) {
    //   if (instance_in_list == instance) {
    //     data_server = reinterpret_cast<DataServer *>(pointer);
    //     break;
    //   }
    // }
    // if (data_server == nullptr) {
    //   return 0;
    // }
    const char *joints_config = mj_getPluginConfig(m, instance, "joints");

    if (!joints_config || strlen(joints_config) == 0) {
      return 0;
    }

    // 计算要返回的关节数量
    if (strcmp(joints_config, "all") == 0) {
      return m->njnt; // 所有关节
    }

    // 计算指定关节的数量
    int count = 0;
    std::string config(joints_config);
    std::istringstream iss(config);
    std::string joint_name;

    while (std::getline(iss, joint_name, ';')) {
      joint_name.erase(0, joint_name.find_first_not_of(' '));
      joint_name.erase(joint_name.find_last_not_of(' ') + 1);

      if (!joint_name.empty()) {
        int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
        if (joint_id >= 0) {
          count++;
        }
      }
    }

    return count;
  };

  // 初始化函数
  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    std::unique_ptr<DataServer> data_server = DataServer::Create(m, instance);
    if (data_server == nullptr) {
      std::cerr << "[DataServer] Failed to create instance" << std::endl;
    }

    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(data_server.release());
    return 0;
  };

  // 销毁函数
  plugin.destroy = +[](mjData *d, int instance) {
    DataServer::Destroy(d, instance);
    d->plugin_data[instance] = 0;
  };

  // 重置函数
  plugin.reset = +[](const mjModel *m, mjtNum *plugin_state, void *plugin_data,
                     int instance) {
    auto *data_server = reinterpret_cast<DataServer *>(plugin_data);
    if (data_server) {
      data_server->Reset(plugin_state);
    }
  };

  // 计算函数
  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        auto *data_server =
            reinterpret_cast<DataServer *>(d->plugin_data[instance]);
        // 当data_server为空时直接返回,表示该实例不对应真实的DataServer对象
        if (data_server == nullptr) {
          return;
        }
        if (instance != data_server->GetPluginInstance()) {
          return;
        }
        if (data_server) {
          if (capability_bit & mjPLUGIN_SENSOR) {
            // 作为传感器计算
            data_server->Compute(m, d, instance);
            int id;
            for (id = 0; id < m->nsensor; ++id) {
              if (m->sensor_type[id] == mjSENS_PLUGIN &&
                  m->sensor_plugin[id] == instance) {
                break;
              }
            }
            auto joints_positions = data_server->GetJointPositions();
            // m->sensor_plugin[instance];
            int sensor_adr = m->sensor_adr[id];
            // mju_zero(sensordata, m->sensor_dim[id]);
            for (size_t i = 0; i < joints_positions.size(); i++) {
              d->sensordata[sensor_adr + i] = joints_positions[i];
            }
            joints_positions.clear();
          }
          if (capability_bit & mjPLUGIN_ACTUATOR) {
            // 作为执行器计算
            {
              std::unique_lock<std::mutex> lock(data_server->command_mutex_);
              data_server->UpdateActuatorControls(m, d);
            }
          }
        }
      };

  // 注册插件
  std::cout << "[DataServer] Registering plugin" << std::endl;
  mjp_registerPlugin(&plugin);
}

} // namespace mujoco::plugin::dataserver