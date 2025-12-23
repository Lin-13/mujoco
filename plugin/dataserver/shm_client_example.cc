// 示例客户端程序：从MuJoCo共享内存读取数据并发送控制命令
#include "shm_client.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
// 用于程序退出释放资源
#include <atomic>
std::atomic<bool> g_running(true);
#ifdef _WIN32
#include <windows.h>
BOOL WINAPI ConsoleHandler(DWORD signal) {
  if (signal == CTRL_C_EVENT || signal == CTRL_CLOSE_EVENT ||
      signal == CTRL_BREAK_EVENT) {
    g_running = false;
  }
  return TRUE;
}
#else
#include <signal.h>
void signalHandler(int signum) { g_running = false; }
#endif
void printDataSummary(const MujocoDataFrame &data_frame) {
  std::cout << "Frame ID: " << data_frame.frame_id
            << ", Timestamp: " << data_frame.timestamp
            << ", Sim Time: " << data_frame.sim_time << "\n";
  std::cout << "Joints: " << data_frame.joints.size()
            << ", Sensors: " << data_frame.sensors.size()
            << ", Bodies: " << data_frame.bodies.size()
            << ", Actuators: " << data_frame.actuators.size() << "\n";
}
void printDataFrame(const MujocoDataFrame &data_frame) {
  std::cout << "=== Data Frame ===" << "\n";
  std::cout << "Description: " << data_frame.desctrption << "\n";
  std::cout << "Frame ID: " << data_frame.frame_id
            << ", Req Frame ID: " << data_frame.req_frame_id
            << ", Timestamp: " << data_frame.timestamp
            << ", Sim Time: " << data_frame.sim_time
            << ", Valid: " << (data_frame.is_valid ? "true" : "false") << "\n";

  // 打印关节数据
  std::cout << "Joints (" << data_frame.joints.size() << "):" << "\n";
  for (const auto &joint : data_frame.joints) {
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
    std::cout << "]" << "\n";
  }

  // 打印传感器数据
  if (!data_frame.sensors.empty()) {
    std::cout << "Sensors (" << data_frame.sensors.size() << "):" << "\n";
    for (const auto &sensor : data_frame.sensors) {
      std::cout << "  " << sensor.name << " [ID:" << sensor.id << "]: [";
      for (size_t i = 0; i < sensor.values.size(); i++) {
        std::cout << sensor.values[i];
        if (i < sensor.values.size() - 1)
          std::cout << ", ";
      }
      std::cout << "]" << "\n";
    }
  }

  // 打印身体数据
  if (!data_frame.bodies.empty()) {
    std::cout << "Bodies (" << data_frame.bodies.size() << "):" << "\n";
    for (const auto &body : data_frame.bodies) {
      std::cout << "  " << body.name << " [ID:" << body.id << "]"
                << "\n";
      std::cout << "    pos: [" << body.position[0] << ", " << body.position[1]
                << ", " << body.position[2] << "]"
                << "\n";
      std::cout << "    quat: [" << body.orientation[0] << ", "
                << body.orientation[1] << ", " << body.orientation[2] << ", "
                << body.orientation[3] << "]" << "\n";
      std::cout << "    lin_vel: [" << body.linear_velocity[0] << ", "
                << body.linear_velocity[1] << ", " << body.linear_velocity[2]
                << "]" << "\n";
      std::cout << "    ang_vel: [" << body.angular_velocity[0] << ", "
                << body.angular_velocity[1] << ", " << body.angular_velocity[2]
                << "]" << "\n";
    }
  }
}
int main(int argc, char *argv[]) {
  std::cout << "MuJoCo Shared Memory Client" << "\n";
// 注册信号处理函数，确保程序退出时能正确释放资源
// 否则可能导致共享内存和同步原语未被正确释放，导致mujoco卡死
#ifdef _WIN32
  SetConsoleCtrlHandler(ConsoleHandler, TRUE);
#else
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
#endif
  // 从命令行参数读取共享内存名称
  std::vector<std::string> shm_names;
  if (argc > 1) {
    shm_names.push_back(argv[1]);
  } else {
    // 默认名称
    shm_names.push_back("global_monitor");
  }

  std::cout << "Connecting to shared memory: " << shm_names[0] << "\n";

  // 创建客户端
  size_t shm_size = 4 * 1024 * 1024; // 4MB
  ShmClient client(shm_names, shm_size);

  // 数据缓冲区
  MujocoDataFrame data_frame;
  int count = 0;
  bool has_prev_frame = false;
  uint64_t last_frame_id = 0;
  int64_t last_timestamp = -1;
  int64_t now_us = 0;
  auto start_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
  uint64_t frame_error_count = 0, timestamp_error_count = 0;
  while (g_running.load()) {
    // 等待新数据（超时1000ms）
    while (!client.IsConnected() && g_running.load()) {
      std::cout << "Waiting for connection to shared memory..." << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      client.Initialize();
    }

    // * 方法1
    // 等待服务端通知并及时接受数据，基于notify同步原语，保证与服务端数据的严格同步
    if (!client.WaitForData(data_frame, 1000)) {
      std::cerr << "WaitForData timeout" << "\n";
      continue;
    }
    // * 方法2
    // 不使用通过轮询接受数据，直接使用锁机制读取数据,可能会丢帧和延迟
    // 因此需要sleep避免阻塞lock
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // if (!client.ReceiveAllData(data_frame)) {
    //   std::cerr << "Failed to receive data" << "\n";
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //   continue;
    // }

    // 获取当前时间戳，检查时间戳是否合理
    now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::system_clock::now().time_since_epoch())
                 .count();
    // 检查timestamp,发出警告
    if (has_prev_frame) {
      uint64_t delay = now_us - data_frame.timestamp;
      if (data_frame.timestamp <= last_timestamp) {
        timestamp_error_count++;
      } else if (data_frame.timestamp - last_timestamp > 1e5) { // 100ms
        timestamp_error_count++;
      } else if ((now_us >= data_frame.timestamp) && (delay > 5e3)) { // 5ms
        timestamp_error_count++;
      }
    }
    last_timestamp = data_frame.timestamp;

    // 丢帧/乱序检测
    if (has_prev_frame) {
      const int64_t delta = static_cast<int64_t>(data_frame.frame_id) -
                            static_cast<int64_t>(last_frame_id);
      if (delta == 1) {
        // 正常
      } else if (delta == 0) {
        frame_error_count++;
      } else if (delta < 0) {
        frame_error_count++;
      } else { // delta > 1
        frame_error_count++;
      }
    }
    has_prev_frame = true;
    last_frame_id = data_frame.frame_id;

    count++;
    // 每100次打印一次数据
    if (count % 100 == 0) {
      std::cout << "====== Data Frame Summary (every 100 frames) ======"
                << std::endl;
      printDataSummary(data_frame);
      auto now = std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count();
      double elapsed_sec = (now - start_time) * 1.0e-6;
      double freq = count / elapsed_sec;
      std::cout << "Elapsed time: " << elapsed_sec
                << " s, Average frequency: " << freq << " Hz" << std::endl;
      std::cout << "Frame errors: " << frame_error_count
                << ", Timestamp errors: " << timestamp_error_count << std::endl;
      frame_error_count = 0;
      timestamp_error_count = 0;
      // printDataFrame(data_frame);
    }
    // 每1次发送一次正弦波控制命令
    if (!data_frame.actuators.empty()) {
      std::unordered_map<std::string, double> commands;
      double time = count * 0.01;
      double cmd_value = std::sin(time * 2.0 * 3.14159265 * 0.5);

      std::string actuator_name =
          data_frame.actuators[0].name; // 使用执行器名称作为执行器名称
      commands[actuator_name] = cmd_value;
      client.SendActuatorCommands(commands);
    }
  }
  std::cout << "Exiting MuJoCo Shared Memory Client" << "\n";
  return 0;
}