/*!
 * @file PeriodicTask.h
 *在单独线程中运行的周期函数的简短实现。
 *周期性任务有一个任务管理器，用于测量它们运行所需的时间。
 */

#ifndef PROJECT_PERIODICTASK_H
#define PROJECT_PERIODICTASK_H

#include <string>
#include <thread>
#include <vector>

class PeriodicTaskManager;

/*!
 * 以给定频率运行单个周期性任务的类
 */
class PeriodicTask 
{
 public:
  PeriodicTask(PeriodicTaskManager* taskManager, float period,std::string name);    //（1）创建单个周期性任务
  void start();
  void stop();
  void printStatus();
  void clearMax();
  bool isSlow();
  virtual void init() = 0;
  virtual void run() = 0;
  virtual void cleanup() = 0;
  virtual ~PeriodicTask() { stop(); }

  float getPeriod() { return _period; }          //获取周期函数运行时间

  float getRuntime() { return _lastRuntime; }    //获取周期函数最后的运行时间点

  float getMaxPeriod() { return _maxPeriod; }    //获取周期函数的最大运行周期

  float getMaxRuntime() { return _maxRuntime; }  //获取周期函数最大运行时间

 private:
  void loopFunction();

  float _period;
  volatile bool _running = false;
  float _lastRuntime = 0;
  float _lastPeriodTime = 0;
  float _maxPeriod = 0;
  float _maxRuntime = 0;
  std::string _name;
  std::thread _thread;
};

/*!
 *监视的周期性任务的类
 */
class PeriodicTaskManager 
{
 public:
  PeriodicTaskManager() = default;
  ~PeriodicTaskManager();
  void addTask(PeriodicTask* task);
  void printStatus();
  void printStatusOfSlowTasks();
  void stopAll();

 private:
  std::vector<PeriodicTask*> _tasks;
};

/*!
 *调用函数实现周期性任务的类
 */
class PeriodicFunction : public PeriodicTask 
{
 public:
  PeriodicFunction(PeriodicTaskManager* taskManager, float period,
                   std::string name, void (*function)())
      : PeriodicTask(taskManager, period, name), _function(function) {}
  void cleanup() {}
  void init() {}
  void run() { _function(); }

  ~PeriodicFunction() { stop(); }

 private:
  void (*_function)() = nullptr;
};

/*!
 *用于打印任务管理器中所有任务的状态的定期任务的类
 */
class PrintTaskStatus : public PeriodicTask 
{
 public:
  PrintTaskStatus(PeriodicTaskManager* tm, float period)
      : PeriodicTask(tm, period, "print-tasks"), _tm(tm) {}
  void run() override { _tm->printStatus(); }

  void init() override {}

  void cleanup() override {}

 private:
  PeriodicTaskManager* _tm;
};

/*!
 * 调用成员函数的周期性任务的类
 */
template <typename T>
class PeriodicMemberFunction : public PeriodicTask 
{
 public:
  PeriodicMemberFunction(PeriodicTaskManager* taskManager, float period,
                         std::string name, void (T::*function)(), T* obj)
      : PeriodicTask(taskManager, period, name),
        _function(function),
        _obj(obj) {}

  void cleanup() {}
  void init() {}                      //用这种方法创建的任务，初始化没有任何操作的
  void run() { (_obj->*_function)(); }//用这种方法创建的任务，循环运行的run()是指定运行创建该任务传进来的函数

 private:
  void (T::*_function)();
  T* _obj;
};

#endif  // PROJECT_PERIODICTASK_H
