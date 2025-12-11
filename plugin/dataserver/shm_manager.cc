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

ShmSync::ShmSync(const std::string &sync_name) : sync_name_(sync_name) {}

ShmSync::~ShmSync() {
#ifdef _WIN32
  if (h_mutex_)
    CloseHandle(h_mutex_);
  if (h_event_)
    CloseHandle(h_event_);
#else
  if (mutex_) {
    pthread_mutex_destroy(mutex_);
    pthread_cond_destroy(cond_);
    munmap(mutex_, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t));
    // 清理共享内存对象（仅创建者需要）
    if (is_mutex_created_) {
      shm_unlink(sync_name_.c_str());
    }
  }
#endif
}

bool ShmSync::lock() {
#ifdef _WIN32
  if (!h_mutex_) {
    h_mutex_ = CreateMutex(NULL, FALSE, sync_name_.c_str());
    if (!h_mutex_)
      return false;
  }
  return WaitForSingleObject(h_mutex_, INFINITE) == WAIT_OBJECT_0;
#else
  if (!mutex_) {
    // Linux：创建共享内存存储互斥锁和条件变量
    int fd = shm_open((sync_name_).c_str(), O_CREAT | O_RDWR, 0666);
    ftruncate(fd, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t));
    mutex_ = (pthread_mutex_t *)mmap(
        NULL, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t),
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    cond_ = (pthread_cond_t *)(mutex_ + 1);

    // 初始化进程间共享的互斥锁/条件变量
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex_, &mutex_attr);

    pthread_condattr_t cond_attr;
    pthread_condattr_init(&cond_attr);
    pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(cond_, &cond_attr);
    is_mutex_created_ = true;
  }
  return pthread_mutex_lock(mutex_) == 0;
#endif
}

bool ShmSync::unlock() {
#ifdef _WIN32
  return ReleaseMutex(h_mutex_) != 0;
#else
  return pthread_mutex_unlock(mutex_) == 0;
#endif
}

bool ShmSync::wait() {
#ifdef _WIN32
  if (!h_event_) {
    h_event_ = CreateEvent(NULL, FALSE, FALSE, (sync_name_ + "_event").c_str());
    if (!h_event_)
      return false;
  }
  return WaitForSingleObject(h_event_, INFINITE) == WAIT_OBJECT_0;
#else
  return pthread_cond_wait(cond_, mutex_) == 0;
#endif
}

bool ShmSync::notify() {
#ifdef _WIN32
  return SetEvent(h_event_) != 0;
#else
  return pthread_cond_signal(cond_) == 0;
#endif
}

void ShmSync::destroy() {
#ifdef _WIN32
  // Windows不需要显式删除
  if (h_mutex_) {
    CloseHandle(h_mutex_);
    h_mutex_ = NULL;
  }
  if (h_event_) {
    CloseHandle(h_event_);
    h_event_ = NULL;
  }
#else
  if (mutex_) {
    pthread_mutex_destroy(mutex_);
    pthread_cond_destroy(cond_);
    munmap(mutex_, sizeof(pthread_mutex_t) + sizeof(pthread_cond_t));
    mutex_ = nullptr;
    cond_ = nullptr;
    // 删除共享内存对象
    shm_unlink(sync_name_.c_str());
    is_mutex_created_ = false;
  }
#endif
}