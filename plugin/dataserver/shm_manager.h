/**
 * @file shm_manager.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-12-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <semaphore.h>
#endif
#include <cstring>
// 跨平台错误码定义
enum class ShmError {
  OK = 0,
  CREATE_FAILED,    // 创建共享内存失败
  OPEN_FAILED,      // 打开共享内存失败
  MMAP_FAILED,      // 内存映射失败
  UNMAP_FAILED,     // 解除映射失败
  DESTROY_FAILED,   // 销毁共享内存失败
  INVALID_PARAM,    // 参数无效
  PERMISSION_DENIED // 权限不足
};
class ShmManager {
public:
  ShmManager(const std::string &shm_name, size_t shm_size);
  ~ShmManager();

  // 禁用拷贝（避免重复映射）
  ShmManager(const ShmManager &) = delete;
  ShmManager &operator=(const ShmManager &) = delete;

  // 创建/打开共享内存（is_create=true：创建，false：仅打开）
  ShmError init(bool is_create = true);

  // 写入数据（offset：偏移量，data：数据指针）
  template <typename T> ShmError write(size_t offset, const T &data) {
    if (offset + sizeof(T) > shm_size_ || shm_ptr_ == nullptr) {
      return ShmError::INVALID_PARAM;
    }
    memcpy(static_cast<char *>(shm_ptr_) + offset, &data, sizeof(T));
    return ShmError::OK;
  }
  template <typename T>
  ShmError write_array(size_t offset, const T *data, size_t count) {
    if (offset + sizeof(T) * count > shm_size_ || shm_ptr_ == nullptr) {
      return ShmError::INVALID_PARAM;
    }
    memcpy(static_cast<char *>(shm_ptr_) + offset, data, sizeof(T) * count);
    return ShmError::OK;
  }
  template <typename T> ShmError read(size_t offset, T &data) {
    if (offset + sizeof(T) > shm_size_ || shm_ptr_ == nullptr) {
      return ShmError::INVALID_PARAM;
    }
    memcpy(&data, static_cast<char *>(shm_ptr_) + offset, sizeof(T));
    return ShmError::OK;
  }
  template <typename T>
  ShmError read_array(size_t offset, T *data, size_t count) {
    if (offset + sizeof(T) * count > shm_size_ || shm_ptr_ == nullptr) {
      return ShmError::INVALID_PARAM;
    }
    memcpy(data, static_cast<char *>(shm_ptr_) + offset, sizeof(T) * count);
    return ShmError::OK;
  }
  void *get_raw_ptr() const { return shm_ptr_; }
  ShmError destroy();

private:
  std::string shm_name_;    // 共享内存名称（Linux：/xxx，Windows：Local\xxx）
  size_t shm_size_;         // 共享内存总大小
  void *shm_ptr_ = nullptr; // 映射后的内存指针
#ifdef _WIN32
  HANDLE h_map_file_ = NULL;
#else
  int shm_fd_ = -1;
#endif
};
class ShmSync {
public:
  ShmSync(const std::string &sync_name);
  ~ShmSync();
  bool lock();
  bool unlock();
  bool wait();
  bool notify();
  void destroy(); // 显式清理共享内存对象

private:
  std::string sync_name_;
#ifdef _WIN32
  HANDLE h_mutex_ = NULL; // 互斥锁
  HANDLE h_event_ = NULL; // 事件（替代条件变量）
#else
  pthread_mutex_t *mutex_ = nullptr; // 进程间互斥锁
  pthread_cond_t *cond_ = nullptr;   // 进程间条件变量
  bool is_mutex_created_ = false;
#endif
};