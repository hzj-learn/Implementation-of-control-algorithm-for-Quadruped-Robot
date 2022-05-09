/*! @file SharedMemory.h
 *  @brief 用于将模拟器程序连接到机器人程序
 */
#ifndef PROJECT_SHAREDMEMORY_H
#define PROJECT_SHAREDMEMORY_H

#include <fcntl.h>    /* For O_* constants */
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <unistd.h>
#include <cassert>
#include <cstring>
#include <stdexcept>
#include <string>
#include "cTypes.h"

#define DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME "development-simulator"

/*!
 *共享内存的POSIX信号量的类。
 */
class SharedMemorySemaphore 
{
 public:
  /*!
    *如果信号量是单位化的，初始化它并设置它的值。这可能是你想打多少次就打多少次。必须至少调用一次。
    *只有一个进程需要调用它，即使它用于多个过程。
    *注意，如果在初始化信号量之后调用init（），则不会改变它的值。
    * @param value  信号量的初始值。
   */
  void init(unsigned int value) 
  {
    if (!_init) 
    {
      if (sem_init(&_sem, 1, value)) 
      {
        printf("[ERROR] Failed to initialize shared memory semaphore: %s\n",
               strerror(errno));
      } 
      else 
      {
        _init = true;
      }
    }
  }

  /*!
   * 增加信号量的值。
   */
  void increment() { sem_post(&_sem); }

  /*!
    *如果信号量的值大于0，则减小该值。
    *否则，等待其值>0，然后递减。
   */
  void decrement() { sem_wait(&_sem); }

  /*!
    *如果信号量的值大于0，则减小该值并返回true
    *否则，返回false（不减量或等待）
   * @return
   */
  bool tryDecrement() { return (sem_trywait(&_sem)) == 0; }

  /*!
    *就像减量，但在等待毫秒后，将放弃
    *如果信号量成功递减，则返回true
   */
  bool decrementTimeout(u64 seconds, u64 nanoseconds) 
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += nanoseconds;
    ts.tv_sec += seconds;
    ts.tv_sec += ts.tv_nsec / 1000000000;
    ts.tv_nsec %= 1000000000;
    return (sem_timedwait(&_sem, &ts) == 0);
  }

  /*!
    *删除信号量。注意，在一个进程中删除一个信号量
    *另一个仍然在使用它会导致非常奇怪的行为。
   */
  void destroy() { sem_destroy(&_sem); }

 private:
  sem_t _sem;
  bool _init = false;
};

/*!
 *存储在共享内存中的对象的容器的类
 *这个对象可以在多个进程或程序中查看
 *注意这里在创建共享内存对象时会有很大的开销，因此，建议两个通信程序应该有一个大的共享内存对象，而不是许多小对象。名称字符串用于标识跨不同程序的共享对象
 *在使用共享内存对象之前，必须先分配新的内存，或将其连接到现有共享内存对象。创建/删除内存可以使用createNew/closeNew完成。查看使用createNew分配的现有对象可以使用附加/分离
 * For an example, see test_sharedMemory.cpp
 */
template <typename T>
class SharedMemoryObject 
{
 public:
  SharedMemoryObject() = default;

  /*!
    *为共享内存对象分配内存并附加到它。
    *如果allowOverwrite为true，并且已经有一个具有此名称的对象，
    *旧对象将被覆盖注意，如果发生这种情况，该对象可能是
    *在非常奇怪的状态下初始化。
    *否则，如果名为的对象已经存在，则抛出std::runtime_error
   */
  bool createNew(const std::string& name, bool allowOverwrite = false) 
  {
    bool hadToDelete = false;
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open new %s, size %ld bytes\n", name.c_str(),
           _size);

    _fd = shm_open(name.c_str(), O_RDWR | O_CREAT,
                   S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
    if (_fd == -1) {
      printf("[ERROR] SharedMemoryObject shm_open failed: %s\n",
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return false;
    }

    struct stat s;
    if (fstat(_fd, &s)) {
      printf("[ERROR] SharedMemoryObject::createNew(%s) stat: %s\n",
             name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return false;
    }

    if (s.st_size) {
      printf(
          "[Shared Memory] SharedMemoryObject::createNew(%s) on something that "
          "wasn't new (size is %ld bytes)\n",
          _name.c_str(), s.st_size);
      hadToDelete = true;
      if (!allowOverwrite)
        throw std::runtime_error(
            "Failed to create shared memory - it already exists.");
      printf("\tusing existing shared memory!\n");
      // return false;
    }

    if (ftruncate(_fd, _size)) {
      printf("[ERROR] SharedMemoryObject::createNew(%s) ftruncate(%ld): %s\n",
             name.c_str(), _size, strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return false;
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) {
      printf("[ERROR] SharedMemory::createNew(%s) mmap fail: %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return false;
    }

  //如果我们在重复使用，共享内存有可能不归零
  //旧记忆。这会引起各种奇怪的问题，特别是如果
  //内存中对象的布局已更改。
    memset(mem, 0, _size);

    _data = (T*)mem;
    return hadToDelete;
  }

  /*!
   * 附加到现有共享内存对象函数
   */
  void attach(const std::string& name) 
  {
    assert(!_data);
    _name = name;
    _size = sizeof(T);
    printf("[Shared Memory] open existing %s size %ld bytes\n", name.c_str(),
           _size);
    _fd = shm_open(name.c_str(), O_RDWR,
                   S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
    if (_fd == -1) {
      printf("[ERROR] SharedMemoryObject::attach shm_open(%s) failed: %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    struct stat s;
    if (fstat(_fd, &s)) {
      printf("[ERROR] SharedMemoryObject::attach(%s) stat: %s\n", name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    if ((size_t)s.st_size != _size) {
      printf(
          "[ERROR] SharedMemoryObject::attach(%s) on something that was "
          "incorrectly "
          "sized (size is %ld bytes, should be %ld)\n",
          _name.c_str(), s.st_size, _size);
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    void* mem =
        mmap(nullptr, _size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
    if (mem == MAP_FAILED) {
      printf("[ERROR] SharedMemory::attach(%s) mmap fail: %s\n", _name.c_str(),
             strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    _data = (T*)mem;
  }

  /*!
    *与当前打开的共享内存对象关联的可用内存函数
    *对象可以使用attach或createNew打开。之后
    *调用此函数时，任何进程都不能使用此共享对象
   */
  void closeNew() 
  {
    assert(_data);
    // first, unmap
    if (munmap((void*)_data, _size)) {
      printf("[ERROR] SharedMemoryObject::closeNew (%s) munmap %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    _data = nullptr;

    if (shm_unlink(_name.c_str())) {
      printf("[ERROR] SharedMemoryObject::closeNew (%s) shm_unlink %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    // close fd
    if (close(_fd)) {
      printf("[ERROR] SharedMemoryObject::closeNew (%s) close %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    _fd = 0;
  }

  /*!
    *关闭当前打开的共享内存对象的此视图。对象
    *可以使用attach或createNew打开。打这个电话之后
    *进程不能再使用这个共享对象，但是其他进程仍然
    *可以。
   */
  void detach() 
  {
    assert(_data);
    // first, unmap
    if (munmap((void*)_data, _size)) {
      printf("[ERROR] SharedMemoryObject::detach (%s) munmap %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    _data = nullptr;

    // close fd
    if (close(_fd)) {
      printf("[ERROR] SharedMemoryObject::detach (%s) close %s\n",
             _name.c_str(), strerror(errno));
      throw std::runtime_error("Failed to create shared memory!");
      return;
    }

    _fd = 0;
  }

  /*!
   * 获取共享内存对象。
   */
  T* get() 
  {
    assert(_data);
    return _data;
  }

  /*!
   * 获取共享内存对象。
   */
  T& operator()() 
  {
    assert(_data);
    return *_data;
  }

 private:
  T* _data = nullptr;
  std::string _name;
  size_t _size;
  int _fd;
};

#endif  // PROJECT_SHAREDMEMORY_H
