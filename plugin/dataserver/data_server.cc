// data_server.cpp
#include "data_server.h"
#include <cstring>
#include <iostream>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <sstream>
#include <vector>
namespace mujoco::plugin::dataserver {

DataServer::DataServer(int port) : port_(port) {
  std::cout << "[DataServer] Initialized on port " << port_ << std::endl;
}

void DataServer::InitializeJointSensors(const mjModel *m,
                                        const char *joints_config) {
  if (!joints_config || strlen(joints_config) == 0) {
    std::cout << "[DataServer] No joints specified for monitoring" << std::endl;
    return;
  }

  // 解析关节配置字符串（格式："joint1;joint2;joint3" 或 "all"）
  std::string config(joints_config);
  std::istringstream iss(config);
  std::string joint_name;

  if (config == "all") {
    // 监控所有关节
    for (int i = 0; i < m->njnt; i++) {
      const char *name = mj_id2name(m, mjOBJ_JOINT, i);
      if (name) {
        joint_ids_.push_back(i);
        joint_names_.push_back(name);
        joint_qpos_adr_.push_back(m->jnt_qposadr[i]);
        std::cout << "[DataServer] Monitoring joint: " << name << " (ID: " << i
                  << ", qpos_adr: " << m->jnt_qposadr[i] << ")" << std::endl;
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
          joint_qpos_adr_.push_back(m->jnt_qposadr[joint_id]);
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

void DataServer::InitializeBodySensors(const mjModel *m,
                                       const char *bodies_config) {
  if (!bodies_config || strlen(bodies_config) == 0) {
    std::cout << "[DataServer] No bodies specified for monitoring" << std::endl;
    return;
  }

  // 解析身体配置字符串（格式："body1;body2;body3" 或 "all"）
  std::string config(bodies_config);
  std::istringstream iss(config);
  std::string body_name;

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
  joint_data_.clear();

  for (size_t i = 0; i < joint_ids_.size(); i++) {
    int joint_id = joint_ids_[i];
    JointData data;
    data.name = joint_names_[i];
    data.id = joint_id;

    // 获取位置
    int qpos_adr = m->jnt_qposadr[joint_id];
    if (qpos_adr >= 0) {
      // 计算这个关节有多少位置坐标
      int nqpos = 0;
      if (joint_id + 1 < m->njnt) {
        nqpos = m->jnt_qposadr[joint_id + 1] - qpos_adr;
      } else {
        nqpos = m->nq - qpos_adr;
      }

      // 获取所有位置坐标
      for (int j = 0; j < nqpos; j++) {
        if (qpos_adr + j < m->nq) {
          data.positions.push_back(d->qpos[qpos_adr + j]);
        }
      }
    }

    // 获取速度
    int dof_adr = m->jnt_dofadr[joint_id];
    if (dof_adr >= 0) {
      // 计算这个关节有多少自由度
      int ndof = 0;
      if (joint_id + 1 < m->njnt) {
        ndof = m->jnt_dofadr[joint_id + 1] - dof_adr;
      } else {
        ndof = m->nv - dof_adr;
      }

      // 获取所有速度
      for (int j = 0; j < ndof; j++) {
        if (dof_adr + j < m->nv) {
          data.velocities.push_back(d->qvel[dof_adr + j]);
        }
      }
    }

    // 获取关节类型
    data.joint_type = m->jnt_type[joint_id];
    joint_data_.push_back(data);
  }
}
void DataServer::GetBodyPoseData(const mjModel *m, const mjData *d) {
  body_positions_.clear();
  body_orientations_.clear();
  body_velocities_.clear();

  for (size_t i = 0; i < body_ids_.size(); i++) {
    int body_id = body_ids_[i];

    // 检查ID是否有效
    if (body_id < 0 || body_id >= m->nbody) {
      continue;
    }

    // 获取位置 (x, y, z)
    int pos_adr = body_xpos_adr_[i];
    if (pos_adr >= 0 && pos_adr + 2 < 3 * m->nbody) {
      body_positions_.push_back(d->xpos[pos_adr]);     // x
      body_positions_.push_back(d->xpos[pos_adr + 1]); // y
      body_positions_.push_back(d->xpos[pos_adr + 2]); // z
    } else {
      body_positions_.push_back(0.0);
      body_positions_.push_back(0.0);
      body_positions_.push_back(0.0);
    }

    // 获取姿态四元数 (w, x, y, z)
    int quat_adr = body_xquat_adr_[i];
    if (quat_adr >= 0 && quat_adr + 3 < 4 * m->nbody) {
      body_orientations_.push_back(d->xquat[quat_adr]);     // w
      body_orientations_.push_back(d->xquat[quat_adr + 1]); // x
      body_orientations_.push_back(d->xquat[quat_adr + 2]); // y
      body_orientations_.push_back(d->xquat[quat_adr + 3]); // z
    } else {
      body_orientations_.push_back(1.0); // w
      body_orientations_.push_back(0.0); // x
      body_orientations_.push_back(0.0); // y
      body_orientations_.push_back(0.0); // z
    }

    // 获取速度 (线速度 + 角速度)
    int velp_adr = body_xvelp_adr_[i];
    int velr_adr = body_xvelr_adr_[i];
    if (velp_adr >= 0 && velp_adr + 2 < 6 * m->nbody && velr_adr >= 0 &&
        velr_adr + 2 < 6 * m->nbody) {
      // 线速度 (vx, vy, vz)
      body_velocities_.push_back(d->cvel[velp_adr]);     // vx
      body_velocities_.push_back(d->cvel[velp_adr + 1]); // vy
      body_velocities_.push_back(d->cvel[velp_adr + 2]); // vz

      // 角速度 (wx, wy, wz)
      body_velocities_.push_back(d->cvel[velr_adr]);     // wx
      body_velocities_.push_back(d->cvel[velr_adr + 1]); // wy
      body_velocities_.push_back(d->cvel[velr_adr + 2]); // wz
    } else {
      body_velocities_.push_back(0.0); // vx
      body_velocities_.push_back(0.0); // vy
      body_velocities_.push_back(0.0); // vz
      body_velocities_.push_back(0.0); // wx
      body_velocities_.push_back(0.0); // wy
      body_velocities_.push_back(0.0); // wz
    }
  }
}

int DataServer::GetNumBodyData() const {
  // 每个身体提供：3个位置 + 4个姿态 + 6个速度 = 13个数据
  return static_cast<int>(body_ids_.size() * 13);
}

void DataServer::InitializeJointActuators(const mjModel *m,
                                          const char *actuators_config) {
  if (!actuators_config || strlen(actuators_config) == 0) {
    std::cout << "[DataServer] No actuators specified for control" << std::endl;
    return;
  }

  // 解析执行器配置字符串（格式："actuator1:gain1;actuator2:gain2"）
  // 当前增益没有被使用
  std::string config(actuators_config);
  std::istringstream iss(config);
  std::string actuator_spec;
  if (config == "all") {
    // 控制所有执行器，默认增益为1.0
    for (int i = 0; i < m->nu; i++) {
      const char *name = mj_id2name(m, mjOBJ_ACTUATOR, i);
      if (name) {
        actuator_ids_.push_back(i);
        actuator_names_.push_back(name);
        actuator_gains_.push_back(1.0); // 默认增益
        std::cout << "[DataServer] Controlling actuator: " << name
                  << " (ID: " << i << ", gain: 1.0)" << std::endl;
      }
    }
    return;
  } else {
    while (std::getline(iss, actuator_spec, ';')) {
      size_t colon_pos = actuator_spec.find(':');
      if (colon_pos != std::string::npos) {
        std::string actuator_name = actuator_spec.substr(0, colon_pos);
        std::string gain_str = actuator_spec.substr(colon_pos + 1);

        // 去除空格
        actuator_name.erase(0, actuator_name.find_first_not_of(' '));
        actuator_name.erase(actuator_name.find_last_not_of(' ') + 1);
        gain_str.erase(0, gain_str.find_first_not_of(' '));
        gain_str.erase(gain_str.find_last_not_of(' ') + 1);

        int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name.c_str());
        if (actuator_id >= 0) {
          double gain = std::stod(gain_str);
          actuator_ids_.push_back(actuator_id);
          actuator_names_.push_back(actuator_name);
          actuator_gains_.push_back(gain);
          std::cout << "[DataServer] Controlling actuator: " << actuator_name
                    << " (ID: " << actuator_id << ", gain: " << gain << ")"
                    << std::endl;
        } else {
          std::cerr << "[DataServer] Warning: Actuator '" << actuator_name
                    << "' not found" << std::endl;
        }
      }
    }
  }
}

std::unique_ptr<DataServer> DataServer::Create(const mjModel *m, int instance) {
  std::cout << "[DataServer] Creating instance " << instance << std::endl;
  // 获取mjModel的结构
  // 读取端口配置

  const char *port_str = mj_getPluginConfig(m, instance, "port");
  int port = port_str ? std::atoi(port_str) : 8080;

  // 创建服务器实例
  auto server = std::make_unique<DataServer>(port);
  server->model_tree_root_ = ModelTreeBuilder::buildTreeFromModel(m);
  server->model_mjcf_ = ModelTreeBuilder::generateMJCF(m);
  std::cout << "[DataServer] Model MJCF:\n" << server->model_mjcf_ << std::endl;
  // 读取关节配置
  const char *joints_config = mj_getPluginConfig(m, instance, "joints");
  server->InitializeJointSensors(m, joints_config);

  // 读取执行器配置
  const char *actuators_config = mj_getPluginConfig(m, instance, "actuators");
  server->InitializeJointActuators(m, actuators_config);
  // 读取身体配置
  const char *bodies_config = mj_getPluginConfig(m, instance, "bodies");
  server->InitializeBodySensors(m, bodies_config);

  // 启动服务器
  // server->StartServer();

  return server;
}

void DataServer::Reset(mjtNum *plugin_state) {
  std::cout << "[DataServer] Reset called" << std::endl;
  // 重置状态
  is_initialized_ = false;
}

void DataServer::Compute(const mjModel *m, mjData *d, int instance) {
  static int count = 0;
  count++;
  // 获取数据并写入到缓冲区
  GetJointData(m, d);
  GetBodyPoseData(m, d);

  // 发送传感器数据（包括关节和身体数据）
  SendData();

  // 接收控制命令
  ReceiveControlCommands();

  // 每100步输出一次信息
  if (count % 100 == 0) {
    std::cout << "[DataServer] Compute called " << count << " times"
              << std::endl;

    // 输出关节角度信息
    for (size_t i = 0; i < joint_ids_.size(); i++) {
      std::cout << "[DataServer] Joint: " << joint_names_[i]
                << " (ID: " << joint_ids_[i] << ")"
                << " Position: " << joint_data_[i].positions[0]
                << " Velocity: " << joint_data_[i].velocities[0] << std::endl;
    }
    // 打印每一个body的数据（用于调试）
    for (size_t i = 0; i < body_ids_.size(); i++) {
      std::cout << "[DataServer] Body: " << body_names_[i]
                << " (ID: " << body_ids_[i] << ")\n";
      std::cout << "  Position: (" << body_positions_[i * 3] << ", "
                << body_positions_[i * 3 + 1] << ", "
                << body_positions_[i * 3 + 2] << ")\n";
      std::cout << "  Orientation (quat): (" << body_orientations_[i * 4]
                << ", " << body_orientations_[i * 4 + 1] << ", "
                << body_orientations_[i * 4 + 2] << ", "
                << body_orientations_[i * 4 + 3] << ")\n";
      std::cout << "  Velocity: (" << body_velocities_[i * 6] << ", "
                << body_velocities_[i * 6 + 1] << ", "
                << body_velocities_[i * 6 + 2] << ", "
                << body_velocities_[i * 6 + 3] << ", "
                << body_velocities_[i * 6 + 4] << ", "
                << body_velocities_[i * 6 + 5] << ")\n";
    }
  }
}

void DataServer::SendData() {
  // TODO: 通过网络发送数据
  // send_joint_data(joint_names_, joint_data_);

  // 发送身体数据
  // send_body_data(body_data);
}

void DataServer::ReceiveControlCommands() {
  // 这里实现接收控制命令的逻辑
  // 例如，从网络客户端接收控制命令并应用到执行器

  // TODO: 从网络接收控制命令
  // std::vector<double> command = receive_from_client();
  std::unordered_map<std::string, double> command; // 示例命令
  // for (auto &actuator_name : actuator_names_) {
  //   command[actuator_name] = 0.0; // 示例：所有命令设为0
  // }
  // 将接收到的命令存入缓冲区
  actuator_commands_.clear();
  for (size_t i = 0; i < actuator_ids_.size(); i++) {
    const std::string &actuator_name = actuator_names_[i];
    if (command.find(actuator_name) != command.end()) {
      actuator_commands_.emplace(actuator_name, command[actuator_name]);
    } else {
      // actuator_commands_.push_back(0.0);
      continue;
    }
  }
}
// 将缓冲区的控制命令应用到mjData
void DataServer::UpdateActuatorControls(const mjModel *m, mjData *d) {
  for (size_t i = 0; i < actuator_ids_.size(); i++) {
    int actuator_id = actuator_ids_[i];
    double gain = actuator_gains_[i];
    if (actuator_commands_.count(actuator_names_[i]) == 0) {
      continue;
    }
    if (actuator_id < m->nu) {
      // 从网络接收控制命令（这里用0作为示例）
      double received_command = actuator_commands_[actuator_names_[i]];

      // 应用增益并设置控制信号
      d->ctrl[actuator_id] = gain * received_command;
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
      "port",      // 端口号
      "joints",    // 要监控的关节（分号分隔，或"all"）
      "actuators", // 要控制的执行器（格式："name:gain;name2:gain2"）
      "bodies"     // 要监控的身体（分号分隔，或"all"）
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
      return -1;
    }

    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(data_server.release());
    return 0;
  };

  // 销毁函数
  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<DataServer *>(d->plugin_data[instance]);
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
        if (data_server) {
          if (capability_bit & mjPLUGIN_SENSOR) {
            // 作为传感器计算
            data_server->Compute(m, d, instance);
          }
          if (capability_bit & mjPLUGIN_ACTUATOR) {
            // 作为执行器计算
            data_server->UpdateActuatorControls(m, d);
          }
        }
      };

  // 注册插件
  std::cout << "[DataServer] Registering plugin" << std::endl;
  mjp_registerPlugin(&plugin);
}

} // namespace mujoco::plugin::dataserver