#include "shm_manager.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

ShmManager::ShmManager(const std::string &shm_name, size_t shm_size)
    : shm_name_(shm_name), shm_size_(shm_size) {}
ShmManager::~ShmManager() {
  if (shm_ptr_) {
#ifdef _WIN32
    UnmapViewOfFile(shm_ptr_);
    if (h_map_file_)
      CloseHandle(h_map_file_);
#else
    munmap(shm_ptr_, shm_size_);
    if (shm_fd_ >= 0)
      close(shm_fd_);
    // 注意：这里不调用shm_unlink，因为可能有其他进程仍在使用
    // 显式清理应该调用destroy()方法
#endif
  }
}

// 初始化：创建/打开共享内存
ShmError ShmManager::init(bool is_create) {
#ifdef _WIN32
  if (is_create) {
    h_map_file_ =
        CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
                          static_cast<DWORD>(shm_size_), shm_name_.c_str());
    if (h_map_file_ == NULL) {
      return GetLastError() == ERROR_ACCESS_DENIED ? ShmError::PERMISSION_DENIED
                                                   : ShmError::CREATE_FAILED;
    }
  } else {
    h_map_file_ =
        OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, shm_name_.c_str());
    if (h_map_file_ == NULL) {
      return ShmError::OPEN_FAILED;
    }
  }
  shm_ptr_ = MapViewOfFile(h_map_file_, FILE_MAP_ALL_ACCESS, 0, 0, shm_size_);
  if (shm_ptr_ == NULL) {
    CloseHandle(h_map_file_);
    h_map_file_ = NULL;
    return ShmError::MMAP_FAILED;
  }
#else
  // Linux/POSIX实现：shm_open/mmap
  int flags = is_create ? (O_CREAT | O_RDWR) : O_RDWR;
  shm_fd_ = shm_open(shm_name_.c_str(), flags, 0666);
  if (shm_fd_ == -1) {
    return errno == EACCES
               ? ShmError::PERMISSION_DENIED
               : (is_create ? ShmError::CREATE_FAILED : ShmError::OPEN_FAILED);
  }
  if (is_create) {
    if (ftruncate(shm_fd_, shm_size_) == -1) {
      close(shm_fd_);
      shm_fd_ = -1;
      return ShmError::CREATE_FAILED;
    }
  }

  // 映射内存
  shm_ptr_ =
      mmap(NULL, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    close(shm_fd_);
    shm_fd_ = -1;
    shm_ptr_ = nullptr;
    return ShmError::MMAP_FAILED;
  }
#endif
  return ShmError::OK;
}

// 销毁共享内存（仅创建者调用）
ShmError ShmManager::destroy() {
  if (shm_ptr_ == nullptr)
    return ShmError::INVALID_PARAM;

#ifdef _WIN32
  // Windows关闭句柄后系统自动回收
  UnmapViewOfFile(shm_ptr_);
  CloseHandle(h_map_file_);
  shm_ptr_ = nullptr;
  h_map_file_ = NULL;
#else
  // Linux删除共享内存文件
  if (shm_unlink(shm_name_.c_str()) == -1) {
    return ShmError::DESTROY_FAILED;
  }
  munmap(shm_ptr_, shm_size_);
  close(shm_fd_);
  shm_ptr_ = nullptr;
  shm_fd_ = -1;
#endif
  return ShmError::OK;
}

ShmSync::ShmSync(const std::string &sync_name, bool is_create)
    : sync_name_(sync_name), is_create_(is_create) {}

ShmSync::~ShmSync() {
#ifdef _WIN32
  if (notify_count_ptr_)
    UnmapViewOfFile(notify_count_ptr_);
  if (h_shm_)
    CloseHandle(h_shm_);
  if (h_mutex_)
    CloseHandle(h_mutex_);
  if (h_cond_)
    CloseHandle(h_cond_);
#else
  if (mutex_) {
    pthread_mutex_destroy(mutex_);
    pthread_cond_destroy(cond_);
    munmap(mutex_, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t) +
                       sizeof(std::atomic<uint32_t>));
    // 清理共享内存对象（仅创建者需要）
    if (is_mutex_created_) {
      shm_unlink(sync_name_.c_str());
    }
  }
#endif
}

#ifdef _WIN32
// Windows: 初始化同步对象（与 Linux 结构一致）
bool ShmSync::init_sync() {
  if (is_initialized_)
    return true;

  // 创建/打开命名互斥锁
  h_mutex_ = CreateMutexA(NULL, FALSE, (sync_name_ + "_mutex").c_str());
  if (!h_mutex_) {
    return false;
  }

  // 创建/打开命名事件（手动重置，用于条件变量）
  h_cond_ = CreateEventA(NULL, TRUE, FALSE, (sync_name_ + "_cond").c_str());
  if (!h_cond_) {
    CloseHandle(h_mutex_);
    h_mutex_ = NULL;
    return false;
  }

  // 创建/打开共享内存用于存储 notify_count
  if (is_create_) {
    h_shm_ = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
                                sizeof(std::atomic<uint32_t>),
                                (sync_name_ + "_state").c_str());
  } else {
    h_shm_ = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE,
                              (sync_name_ + "_state").c_str());
  }

  if (!h_shm_) {
    CloseHandle(h_mutex_);
    CloseHandle(h_cond_);
    h_mutex_ = NULL;
    h_cond_ = NULL;
    return false;
  }

  notify_count_ptr_ = static_cast<std::atomic<uint32_t> *>(MapViewOfFile(
      h_shm_, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(std::atomic<uint32_t>)));

  if (!notify_count_ptr_) {
    CloseHandle(h_mutex_);
    CloseHandle(h_cond_);
    CloseHandle(h_shm_);
    h_mutex_ = NULL;
    h_cond_ = NULL;
    h_shm_ = NULL;
    return false;
  }

  // 如果是创建者，初始化 notify_count
  if (is_create_) {
    new (notify_count_ptr_) std::atomic<uint32_t>(0);
  }

  is_initialized_ = true;
  return true;
}
#endif

bool ShmSync::lock() {
#ifdef _WIN32
  if (!is_initialized_) {
    if (!init_sync())
      return false;
  }
  return WaitForSingleObject(h_mutex_, INFINITE) == WAIT_OBJECT_0;
#else
  if (!mutex_) {
    // Linux：创建共享内存存储互斥锁和条件变量
    size_t shm_size = sizeof(pthread_mutex_t) + sizeof(pthread_cond_t) +
                      sizeof(std::atomic<uint32_t>);
    int flags = is_create_ ? (O_CREAT | O_RDWR) : O_RDWR;
    int fd = shm_open((sync_name_).c_str(), flags, 0666);
    if (fd == -1) {
      return false;
    }
    if (is_create_) {
      if (ftruncate(fd, shm_size) != 0) {
        close(fd);
        return false;
      }
    }
    mutex_ = (pthread_mutex_t *)mmap(NULL, shm_size, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd, 0);
    close(fd);
    if (mutex_ == MAP_FAILED) {
      mutex_ = nullptr;
      return false;
    }
    cond_ = (pthread_cond_t *)(mutex_ + 1);
    notify_count_ptr_ = (std::atomic<uint32_t> *)(cond_ + 1);

    if (is_create_) {
      // 初始化进程间共享的互斥锁/条件变量
      pthread_mutexattr_t mutex_attr;
      pthread_mutexattr_init(&mutex_attr);
      pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
      pthread_mutex_init(mutex_, &mutex_attr);

      pthread_condattr_t cond_attr;
      pthread_condattr_init(&cond_attr);
      pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
      pthread_cond_init(cond_, &cond_attr);

      // 初始化notify_count
      new (notify_count_ptr_) std::atomic<uint32_t>(0);

      is_mutex_created_ = true;
    }
  }
  int ret = pthread_mutex_lock(mutex_);
  return ret == 0;
#endif
}

bool ShmSync::unlock() {
#ifdef _WIN32
  return ReleaseMutex(h_mutex_) != 0;
#else
  int ret = pthread_mutex_unlock(mutex_);
  return ret == 0;
#endif
}

// 合并wait和waitfor，timeout_ms<1时为无限等待，否则为超时等待
// 与cv一样需要先lock
// 通过检查原子化的notify_count是否变化来避免虚假唤醒
bool ShmSync::wait(int timeout_ms) {
#ifdef _WIN32
  if (!is_initialized_) {
    if (!init_sync())
      return false;
  }

  // 记录 wait 前的 notify_count，用于检测虚假唤醒
  uint32_t count_before =
      notify_count_ptr_ ? notify_count_ptr_->load(std::memory_order_relaxed)
                        : 0;

  DWORD wait_time =
      (timeout_ms < 1) ? INFINITE : static_cast<DWORD>(timeout_ms);

  if (timeout_ms < 1) {
    // 无限等待：循环直到 notify_count 变化
    while (true) {
      // 释放互斥锁
      ReleaseMutex(h_mutex_);
      // 等待事件
      DWORD ret = WaitForSingleObject(h_cond_, INFINITE);
      // 重新获取互斥锁
      WaitForSingleObject(h_mutex_, INFINITE);

      if (ret != WAIT_OBJECT_0)
        return false;

      // 检查是否真正被 notify，而非虚假唤醒
      if (!notify_count_ptr_ ||
          notify_count_ptr_->load(std::memory_order_relaxed) != count_before) {
        return true;
      }
    }
  } else {
    // 超时等待：循环直到 notify_count 变化或超时
    while (true) {
      // 释放互斥锁
      ReleaseMutex(h_mutex_);
      // 等待事件
      DWORD ret = WaitForSingleObject(h_cond_, wait_time);
      // 重新获取互斥锁
      WaitForSingleObject(h_mutex_, INFINITE);

      if (ret == WAIT_TIMEOUT) {
        return false; // 超时
      }
      if (ret != WAIT_OBJECT_0) {
        return false;
      }
      // 检查是否真正被 notify
      if (!notify_count_ptr_ ||
          notify_count_ptr_->load(std::memory_order_relaxed) != count_before) {
        return true;
      }
    }
  }
#else
  // 记录 wait 前的 notify_count，用于检测虚假唤醒
  uint32_t count_before =
      notify_count_ptr_ ? notify_count_ptr_->load(std::memory_order_relaxed)
                        : 0;
  if (timeout_ms < 1) {
    // 无限等待：循环直到 notify_count 变化
    while (true) {
      int ret = pthread_cond_wait(cond_, mutex_);
      if (ret != 0)
        return false;
      // 检查是否真正被 notify，而非虚假唤醒
      if (!notify_count_ptr_ ||
          notify_count_ptr_->load(std::memory_order_relaxed) != count_before) {
        return true;
      }
    }
  } else {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
      ts.tv_sec += 1;
      ts.tv_nsec -= 1000000000;
    }
    // 超时等待：循环直到 notify_count 变化或超时
    while (true) {
      int ret = pthread_cond_timedwait(cond_, mutex_, &ts);
      if (ret == ETIMEDOUT) {
        return false; // 超时
      }
      if (ret != 0) {
        return false;
      }
      // 检查是否真正被 notify
      if (!notify_count_ptr_ ||
          notify_count_ptr_->load(std::memory_order_relaxed) != count_before) {
        return true;
      }
    }
  }
#endif
}

bool ShmSync::notify() {
#ifdef _WIN32
  if (!is_initialized_) {
    if (!init_sync())
      return false;
  }
  if (notify_count_ptr_) {
    notify_count_ptr_->fetch_add(1, std::memory_order_relaxed);
  }
  // 唤醒一个等待者
  PulseEvent(h_cond_);
  return true;
#else
  if (notify_count_ptr_) {
    notify_count_ptr_->fetch_add(1, std::memory_order_relaxed);
  }
  int ret = pthread_cond_signal(cond_);
  return ret == 0;
#endif
}

bool ShmSync::notify_all() {
#ifdef _WIN32
  if (!is_initialized_) {
    if (!init_sync())
      return false;
  }
  if (notify_count_ptr_) {
    notify_count_ptr_->fetch_add(1, std::memory_order_relaxed);
  }
  // 唤醒所有等待者
  PulseEvent(h_cond_);
  return true;
#else
  if (notify_count_ptr_) {
    notify_count_ptr_->fetch_add(1, std::memory_order_relaxed);
  }
  int ret = pthread_cond_broadcast(cond_);
  return ret == 0;
#endif
}

void ShmSync::destroy() {
#ifdef _WIN32
  if (notify_count_ptr_) {
    UnmapViewOfFile(notify_count_ptr_);
    notify_count_ptr_ = nullptr;
  }
  if (h_shm_) {
    CloseHandle(h_shm_);
    h_shm_ = NULL;
  }
  if (h_mutex_) {
    CloseHandle(h_mutex_);
    h_mutex_ = NULL;
  }
  if (h_cond_) {
    CloseHandle(h_cond_);
    h_cond_ = NULL;
  }
  is_initialized_ = false;
#else
  if (mutex_) {
    pthread_mutex_destroy(mutex_);
    pthread_cond_destroy(cond_);
    munmap(mutex_, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t) +
                       sizeof(std::atomic<uint32_t>));
    mutex_ = nullptr;
    cond_ = nullptr;
    notify_count_ptr_ = nullptr;
    // 删除共享内存对象
    shm_unlink(sync_name_.c_str());
    is_mutex_created_ = false;
  }
#endif
}