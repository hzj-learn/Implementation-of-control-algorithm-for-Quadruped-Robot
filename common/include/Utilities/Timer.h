/*! @file Timer.h
 *  @brief 测量事物所需时间的计时器
 */

#ifndef PROJECT_TIMER_H
#define PROJECT_TIMER_H

#include <assert.h>
#include <stdint.h>
#include <time.h>

class Timer 
{
 public:
  explicit Timer()        //（1）定义计时器
  { start(); }

  void start()            //（2）计时器开始计时
  { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  double getMs()          //（3）计时器获取毫秒级时间数据
  { return (double)getNs() / 1.e6; }

  int64_t getNs()         //（4）计时器获取纳秒级时间数据
  {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  double getSeconds()     //（5）计时器获取秒级时间数据
  { return (double)getNs() / 1.e9; }

  struct timespec _startTime;
};

#endif  // PROJECT_TIMER_H
