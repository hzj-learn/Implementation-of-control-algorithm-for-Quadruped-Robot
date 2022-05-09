/*!
 * @file PeriodicTask.cpp
 * @brief 在单独线程中运行的周期函数的实现。
 * 周期性任务有一个任务管理器，用于测量它们运行所需的时间。
 */

#include "include/Utilities/PeriodicTask.h"
#include <include/Utilities/Timer.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <cmath>
#include "Utilities/Utilities_print.h"

/*!
 * 功能：创建单个周期性任务
 * 备注：创建一个任务格式：fun（任务管理器,周期，任务名称，运行的函数，对象）
 * 正是用这个函数创建了7个机器人任务的
 */
PeriodicTask::PeriodicTask(PeriodicTaskManager* taskManager, float period,
                           std::string name)
    : _period(period), _name(name) 
{
  taskManager->addTask(this);
}



/*!
 * 功能：开始运行单个周期任务
 */
void PeriodicTask::start() 
{
  if (_running) //若该任务已经运行了
  {
    printf("[PeriodicTask] Tried to start %s but it was already running!\n",
           _name.c_str());
    return;
  }
  _running = true;//表示正在运行该任务
  init();                                                  //（1）初始化该任务，这里的init()是实例化到不同的类的init()的
  _thread = std::thread(&PeriodicTask::loopFunction, this);//（2）开线程周期运行loopFunction这个函数
}


/*!
 * 功能：停止运行单个周期任务
 */
void PeriodicTask::stop() 
{
  if (!_running) //若该任务已经停止运行了
  {
    printf("[PeriodicTask] Tried to stop %s but it wasn't running!\n",
           _name.c_str());
    return;
  }
  _running = false;//停止该任务
  printf("[PeriodicTask] Waiting for %s to stop...\n", _name.c_str());
  _thread.join();   //退出线程
  printf("[PeriodicTask] Done!\n");
  cleanup();
}


/*!
 * 功能：周期任务运行时间显示
 */
bool PeriodicTask::isSlow() 
{
  return _maxPeriod > _period * 1.3f || _maxRuntime > _period;
}


/*!
 * 功能：清除任务最大的运行周期和运行时间
 */
void PeriodicTask::clearMax() 
{
  _maxPeriod = 0;
  _maxRuntime = 0;
}


/*!
 * 功能：打印周期任务的运行时间
 */
void PeriodicTask::printStatus() 
{
  if (!_running) return;//若任务已经停止了

  if (isSlow()) 
  {
    printf_color(PrintColor::Red, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n",
                 _name.c_str(), _lastRuntime, _maxRuntime, _period,
                 _lastPeriodTime, _maxPeriod);
  } 
  else 
  {
    printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", _name.c_str(),
           _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod);
  }
}


/*!
 * 功能：周期循环函数
 */
void PeriodicTask::loopFunction() 
{
  /*（1）定义一下计时变量和定时器*/
  auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);    //库函数，创建单调时钟
  int seconds = (int)_period;                           //以秒为计量单位
  int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
  Timer t;                                  //创建一个定时器
  itimerspec timerSpec;
  timerSpec.it_interval.tv_sec = seconds;
  timerSpec.it_value.tv_sec = seconds;
  timerSpec.it_value.tv_nsec = nanoseconds;
  timerSpec.it_interval.tv_nsec = nanoseconds;
  timerfd_settime(timerFd, 0, &timerSpec, nullptr);
  unsigned long long missed = 0;

  /*（2）开始运行死循环的周期run()函数，并开始运行计时器*/
  printf("[PeriodicTask] Start %s (%d s, %d ns)\n", _name.c_str(), seconds,
         nanoseconds);
  while (_running)          //周期函数已经启动了
  {
    _lastPeriodTime = (float)t.getSeconds();//
    t.start();
    run();                                      // 【真他太重要了！运行void RobotRunner::run()】
    _lastRuntime = (float)t.getSeconds();
    int m = read(timerFd, &missed, sizeof(missed));
    (void)m;
    _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);//计算进程的最大周期
    _maxRuntime = std::max(_maxRuntime, _lastRuntime); //计算进程的最大运行时间
  }

  /*(3)当周期循环函数用标志位关闭后，线程自动退出*/
  printf("[PeriodicTask] %s has stopped!\n", _name.c_str());
}

PeriodicTaskManager::~PeriodicTaskManager() {}


/*!
 * 功能：增加一个周期函数
 */
void PeriodicTaskManager::addTask(PeriodicTask* task) 
{
  _tasks.push_back(task);
}


/*!
 * 功能：打印周期任务管理器状态
 */
void PeriodicTaskManager::printStatus() 
{
  /*liu
  printf("\n----------------------------TASKS----------------------------\n");
  printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des",
         "T-act", "T-max");
  printf("-----------------------------------------------------------\n");
  for (auto& task : _tasks) {
    task->printStatus();
    task->clearMax();
  }
  printf("-------------------------------------------------------------\n\n");
  liu*/
}


/*!
 * 功能：打印周期任务管理器中运行的任务
 */
void PeriodicTaskManager::printStatusOfSlowTasks() 
{
  for (auto& task : _tasks) {
    if (task->isSlow()) {
      task->printStatus();
      task->clearMax();
    }
  }
}

/*!
 * 功能：停止所有的周期函数
 */
void PeriodicTaskManager::stopAll() 
{
  for (auto& task : _tasks) {
    task->stop();
  }
}
