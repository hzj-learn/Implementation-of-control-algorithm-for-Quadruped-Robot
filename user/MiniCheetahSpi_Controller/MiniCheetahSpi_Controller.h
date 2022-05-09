#ifndef CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H
#define CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H

#include <RobotController.h>
#include <lcm/lcm-cpp.hpp>

#include <thread>
#include <mutex>


//MiniCheetahSpi_Controller这个类是继承RobotController这个类的
class MiniCheetahSpi_Controller : public RobotController 
{
public:
  MiniCheetahSpi_Controller():RobotController(), _lcm(getLcmUrl(255)) 
  {
    _lcm.subscribe("spi_debug_cmd", &MiniCheetahSpi_Controller::handleLcm, this);//（1）通过LCM创建订阅SPI任务（spi_debug_cmd）
                                                                                 //（2）开启LCM线程，中断订阅通讯板发过来的SPI数据
    _lcmThread = std::thread([&]()
    {
      for(;;) _lcm.handle();
    });
  }

  virtual ~MiniCheetahSpi_Controller(){}
  virtual void initializeController(){}
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() 
  {
    return nullptr;
  }
//这个程序是SPI中断处理程序（接收数据）
  void handleLcm(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const leg_control_command_lcmt* msg) 
  {
    (void)rbuf;
    (void)chan;
    _mutex.lock();    //线程上锁，进入临界区
    command = *msg;   //接收搬运msg缓存数据到spi命令数组中
    _mutex.unlock();  //线程解锁，退出临界区
  }

private:
  lcm::LCM _lcm;
  leg_control_command_lcmt command;
  std::thread _lcmThread;
  std::mutex _mutex;
};

#endif //CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H
