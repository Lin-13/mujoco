// 示例客户端程序：从MuJoCo共享内存读取数据并发送控制命令
#include "shm_client.h"
#include <chrono>
#include <iostream>
#include <thread>
int main(int argc, char *argv[]) {
  std::cout << "MuJoCo Shared Memory Client" << std::endl;

  // 从命令行参数读取共享内存名称
  std::vector<std::string> shm_names;
  if (argc > 1) {
    shm_names.push_back(argv[1]);
  } else {
    // 默认名称
    shm_names.push_back("global_monitor");
  }

  std::cout << "Connecting to shared memory: " << shm_names[0] << std::endl;

  // 创建客户端
  size_t shm_size = 4 * 1024 * 1024; // 4MB
  ShmClient client(shm_names, shm_size);

  // 数据缓冲区
  std::vector<JointData> joint_data;
  std::vector<SensorData> sensor_data;
  std::vector<PoseData> body_data;
  std::vector<ActuatorData> actuator_data;
  int count = 0;
  while (true) {
    // 等待新数据（超时1000ms）
    while (!client.IsConnected()) {
      std::cout << "Waiting for connection to shared memory..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      client.Initialize();
    }
    // 接收所有数据
    if (!client.ReceiveAllData(joint_data, sensor_data, body_data,
                               actuator_data)) {
      std::cerr << "Failed to receive data" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    count++;
    // 每100次打印一次数据
    if (count % 100 == 0) {
      std::cout << "\n=== Data Frame " << count << " ===" << std::endl;

      // 打印关节数据
      std::cout << "Joints (" << joint_data.size() << "):" << std::endl;
      for (const auto &joint : joint_data) {
        std::cout << "  " << joint.name << " [ID:" << joint.id << "]";
        std::cout << " pos:[";
        for (size_t i = 0; i < joint.positions.size(); i++) {
          std::cout << joint.positions[i];
          if (i < joint.positions.size() - 1)
            std::cout << ", ";
        }
        std::cout << "] vel:[";
        for (size_t i = 0; i < joint.velocities.size(); i++) {
          std::cout << joint.velocities[i];
          if (i < joint.velocities.size() - 1)
            std::cout << ", ";
        }
        std::cout << "]" << std::endl;
      }

      // 打印传感器数据
      if (!sensor_data.empty()) {
        std::cout << "Sensors (" << sensor_data.size() << "):" << std::endl;
        for (const auto &sensor : sensor_data) {
          std::cout << "  " << sensor.name << " [ID:" << sensor.id << "]: [";
          for (size_t i = 0; i < sensor.values.size(); i++) {
            std::cout << sensor.values[i];
            if (i < sensor.values.size() - 1)
              std::cout << ", ";
          }
          std::cout << "]" << std::endl;
        }
      }

      // 打印身体数据
      if (!body_data.empty()) {
        std::cout << "Bodies (" << body_data.size() << "):" << std::endl;
        for (const auto &body : body_data) {
          std::cout << "  " << body.name << " [ID:" << body.id << "]"
                    << std::endl;
          std::cout << "    pos: [" << body.position[0] << ", "
                    << body.position[1] << ", " << body.position[2] << "]"
                    << std::endl;
          std::cout << "    quat: [" << body.orientation[0] << ", "
                    << body.orientation[1] << ", " << body.orientation[2]
                    << ", " << body.orientation[3] << "]" << std::endl;
          std::cout << "    lin_vel: [" << body.linear_velocity[0] << ", "
                    << body.linear_velocity[1] << ", "
                    << body.linear_velocity[2] << "]" << std::endl;
          std::cout << "    ang_vel: [" << body.angular_velocity[0] << ", "
                    << body.angular_velocity[1] << ", "
                    << body.angular_velocity[2] << "]" << std::endl;
        }
      }
    }
    // 每1次发送一次正弦波控制命令
    if (!actuator_data.empty()) {
      std::unordered_map<std::string, double> commands;
      double time = count * 0.01;
      double cmd_value = std::sin(time * 2.0 * 3.14159265 * 0.5);

      std::string actuator_name =
          actuator_data[0].name; // 使用执行器名称作为执行器名称
      commands[actuator_name] = cmd_value;

      std::cout << "\nSending command: " << actuator_name << " = " << cmd_value
                << std::endl;
      client.SendActuatorCommands(commands);
    }
    // 控制循环频率
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}