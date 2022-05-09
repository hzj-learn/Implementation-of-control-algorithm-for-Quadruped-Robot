/*!
 * @file RobotRunner.cpp
 * @brief 文件功能：运行机器人控制器的通用框架，此代码是mini cheetah和cheetah 3的控制代码和硬件/仿真之间的通用接口
 */

#include <unistd.h>
#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"


/*
 *功能:机器人运行器,将运行框架加入任务管理器
 *用法：在hardwareBridge使用，实例化运行器 传入控制器，任务管理器 参数 名称 
 *  _robotRunner =new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");
*/
RobotRunner::RobotRunner(RobotController* robot_ctrl, PeriodicTaskManager* manager, float period, std::string name):
  PeriodicTask(manager, period, name),//添加任务
  _lcm(getLcmUrl(255)) 
  {
    _robot_ctrl = robot_ctrl;//目前为空 之后应该会定义
  }

/*
 *功能:RobotRunner()的析构函数 
*/
RobotRunner::~RobotRunner() 
{
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}


/*
 *功能：创建机器人模型，状态估计器，腿部控制器，机器人数据，以及任何控制逻辑特定的数据
 *步骤：
    （1）选择机器人类型
    （2）初始化模型和机器人数据
    （3）初始化腿控制器和状态估计器
    （4）初始化所需的状态命令对象
    （5）控制器参数初始化
    【用于创建机器人各种控制器】
 */
void RobotRunner::init() 
{
  printf("[RobotRunner] initialize\n");
  /*（1）选择机器人类型*/
  if (robotType == RobotType::MINI_CHEETAH) //选择MINI_CHEETAH
  {
    _quadruped = buildMiniCheetah<float>();
  }
  else                                     //选择CHEETAH 3
  {
    _quadruped = buildCheetah3<float>();
  }

  /*（2）创建机器人模型*/
  _model = _quadruped.buildModel();  //建立四足动物的浮基模型

  /*（3）创建关节控制器*/
  _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);//关节控制器

  /*（4）创建腿控制器*/
    _legController  = new LegController<float>(_quadruped);    //腿控制器

  /*（5）创建一系列状态估计器*/
  _stateEstimator = new StateEstimatorContainer<float>(      //1）定义总的状态估计器，fun(仿真模式、imu导航数据、腿部控制器数据、状态估计结构体、机器人控制参数)
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);                          //2）创建一系列子状态估计器
  
  /*（6）创建期望状态参考COM轨迹规划器*/
  memset(&rc_control, 0, sizeof(rc_control_settings));//把遥控器的设置指令复制到变量rc_control中
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        &rc_control,
        controlParameters,
        &_stateEstimate,
        controlParameters->controller_dt);

  /*（7）设置机器人控制参数类型，并初始化机器人控制器*/
  //先设置机器人控制参数
  _robot_ctrl->_model = &_model;                            //1）模型控制参数类型
  _robot_ctrl->_quadruped = &_quadruped;                    //2）四足机器人物理特性的表征控制参数类型
  _robot_ctrl->_legController = _legController;             //3）腿部控制器控制参数类型
  _robot_ctrl->_stateEstimator = _stateEstimator;           //4）状态估计器控制参数类型
  _robot_ctrl->_stateEstimate = &_stateEstimate;            //5）状态估计值控制参数类型
  _robot_ctrl->_visualizationData= visualizationData;       //6）可视化数据控制参数类型
  _robot_ctrl->_robotType = robotType;                      //7）机器人类型控制参数类型
  _robot_ctrl->_driverCommand = driverCommand;              //8）驱动命令控制参数类型
  _robot_ctrl->_controlParameters = controlParameters;      //9）控制参数控制参数类型
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand; //10）期望状态命令控制参数类型
  
  //后初始化机器人控制器，运行的是RobotController类内的initializeController
  //分别是注意这里的控制器对应user文件夹的5个控制器
  //1)JPos_Controller
  //2)Leg_InvDyn_Controller
  //3)MiniCheetahSpi_Controller
  //4)MIT_Controller
  //5)WBC_Controller
  _robot_ctrl->initializeController();                      //11）初始化控制器【重要】
}


/*
  *功能：通过调用每个主要组件来运行整个机器人控制系统。运行它们各自的步骤。运行周期从参数文件来
  *步骤：
  * （1）运行状态估计器,仿真可视化清零
  * （2）更新来自机器人的数据
  * （3）开启腿控制器
  * （4）写入四条腿12个关节可视化的数据
  * （5）先把四条腿的位置p可视化复位置零
  * （6）再把状态估计器估计的位置p,写入可视化位置p的数据里
  * （7）用状态估计器的方向数据，写入可视化的方向数据quat中
  * （8）为机器人设置适当的腿部控制器命令，更新command
}
 */
//在周期任务中会反复运行这个函数
void RobotRunner::run() 
{
  /////////////*（1）运行所有的状态估计器,仿真可视化清零*//////////////////////////
  //（1）前面初始化的时候开启运行IMU线程，读到了加速度、四元素（转换成RPY）、陀螺仪数据
  //（1）现在运行所有的状态估计器，包括1）方向估计器、2）位置速度估计器、3）触地估计器
  _stateEstimator->run();  //注意：这里是运行StateEstimatorContainer类内的run()函数

  //（2）仿真可视化数据清零                              
  visualizationData->clear(); 
  


  ////////////*（2）开机硬件设备数据、命令、标志位配置*///////////////////////////
  setupStep();              //在运行用户代码之前，设置腿控件和估计器，RobotRunner的类内函数，所以跳转不过去的      

 /////////////*（3）开启腿控制器*///////////////////////////////////////////////
  static int count_ini(0);
  ++count_ini;
  if (count_ini < 10)                    //关闭腿控制器
  {
    _legController->setEnabled(false);
  } 
  else if (20 < count_ini && count_ini < 30)  //关闭腿控制器
  {
    _legController->setEnabled(false);
  }
  else if (40 < count_ini && count_ini < 50)  //关闭腿控制器
  {
    _legController->setEnabled(false);
  }
  else   
  {
    _legController->setEnabled(true);//开启腿部控制器

  /*若手柄连接出现问题：不在手柄口至模式，并在使用用户仿真界面的参数 ，进行错误处理*/
    if( (rc_control.mode == 0) && controlParameters->use_rc )
     {  
      if(count_ini%1000 ==0)   printf("ESTOP!\n");           //（1）超时，打印错误

      for (int leg = 0; leg < 4; leg++)                      //（2）四条腿命令复位清零
      {
        _legController->commands[leg].zero();
      }
      _robot_ctrl->Estop();                                  //（3）停止机器人控制
     }
  /*若手柄连接没有问题，则正常运行机器人*/
    else                                                    
     {
      if (!_jpos_initializer->IsInitialized(_legController)) //仅仅运行一次，关节初始化，按照B样条曲线平稳移动跟踪到匍匐状态的函数后，写入并反馈增益矩阵到关节控制器【看到这里】
      {
        Mat3<float> kpMat;                       //定义反馈增益KP 3*3矩阵
        Mat3<float> kdMat;                       //定义反馈增益KD 3*3矩阵
        if (robotType == RobotType::MINI_CHEETAH)//先写入MINI_CHEETAH的jpos反馈增益
         {
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
         } 

        else if (robotType == RobotType::CHEETAH_3)//先写入CHEETAH 3 的jpos反馈增益
         {
          kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
          kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
         }

        else                                     //既不是MINI_CHEETAH，也不是CHEETAH_3，报错
         {
          assert(false);
         } 

        for (int leg = 0; leg < 4; leg++)        //再更新jpos反馈增益到腿部控制器
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      } 
      else                                                   //若关节没有经过了平稳移动，运行控制器，并更新可视化，之后就运行运行这个了
      {
         //（1）【重点！！！】按照之前传进来的控制器，分别运行MiniCheetahSpi_Controller、JPos_Controller、MIT_Controller、WBC_Controller控制器对应的runController()函数
        _robot_ctrl->runController();                            
        cheetahMainVisualization->p = _stateEstimate.position;    //（2）写入状态估计器估计的位置
        _robot_ctrl->updateVisualization();                       //（3）更新可视化数据
        cheetahMainVisualization->p = _stateEstimate.position;    //（4）再/写入状态估计器估计的位置
      }
    }
  }

   ////////////* (4)写入四条腿12个关节可视化的数据*/////////////////////////////////
  for (int leg = 0; leg < 4; leg++)     
  {
    for (int joint = 0; joint < 3; joint++) 
    {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }


  /////////////*（5）先把四条腿的位置p可视化复位置零*//////////////////////////////
  cheetahMainVisualization->p.setZero();   

  /*（6）再把状态估计器估计的位置p,写入可视化位置p的数据里
         这里的状态估计的结果会不断的更新
         因为状态估计是在周期函数里面不断运行的*/
  cheetahMainVisualization->p = _stateEstimate.position;   

/*（7）用状态估计器的方向数据，写入可视化的方向数据quat中*/
  cheetahMainVisualization->quat = _stateEstimate.orientation;
  
/*（8）为机器人设置适当的腿部控制器命令，更新command，相当于电机输出*/  
  finalizeStep();                                           
}


/*!
  * 功能：开机硬件设备配置，开启线程
  * 注意：在运行用户代码之前调用这个函数
  * 步骤：
  * 设置腿控件和估计器，用于安全检查
  * （1）更新腿部数据
  * （2）为新的迭代设置腿控制器
  * （3）转换到检查（Cheater）模式，用于安全检查，或者转换到机器人模式
  * （4）获得手柄控制设置
 */
void RobotRunner::setupStep() 
{
  ////////////////////////*（1）更新腿部控制器SPI数据*//////////////////////////////////////////////////////
  if (robotType == RobotType::MINI_CHEETAH) //MINI_CHEETAH更新腿部数据 
  {
    _legController->updateData(spiData);    //更新 从tiBoardData来的值 到datas
  } 
  else if (robotType == RobotType::CHEETAH_3)    //CHEETAH_3 更新腿部数据 
  {
    _legController->updateData(tiBoardData);//更新 从tiBoardData来的值 到datas
  } 
  else                                      //既不是MINI_CHEETAH，也不是CHEETAH_3，不更新腿部数据，报错
  {
    assert(false);
  }

  //////////////////////*（2）腿部控制器命令清零、设置关节最大力矩后开启腿部控制线程*////////////////////////////////////////////////
  _legController->zeroCommand();              //腿部控制命令清零
  _legController->setEnabled(true);           //开启腿部控制线程【重要！！！】
  _legController->setMaxTorqueCheetah3(208.5);//设置关节最大力矩

  //////////////////////*（3）初始化状态估计器线程*//////////////////////////////
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) 
  {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);                 //初始化状态估计器线程，状态估计器再周期任务中不断运行
    // todo any configuration
    _cheaterModeEnabled = true;                     //启动机器人控制
  }

  if (_cheaterModeEnabled && !controlParameters->cheater_mode) 
  {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);               //仿真状态，初始化状态估计器
    // todo any configuration
    _cheaterModeEnabled = false;                   //关闭机器人控制
  }
 
 ////////////////////////*（4）获得手柄控制设置*///////////////////////////////////////////////////
  get_rc_control_settings(&rc_control);
  // todo safety checks, sanity checks, etc...
}


/*!
  * 功能：在用户代码之后，发送腿命令，更新状态估计，并发布调试数据
  * 步骤：
  * （1）更新机器人的SPI命令
  * （2）设置腿部控制器和状态估计器的数据到LCM，再使用LCM发布出去
 */
void RobotRunner::finalizeStep() 
{
  /*（1）更新机器人的SPI命令*/
  if (robotType == RobotType::MINI_CHEETAH)         //更新MINI_CHEETAH的SPI命令
   {
    _legController->updateCommand(spiCommand);
   } 
  else if (robotType == RobotType::CHEETAH_3)       //更新CHEETAH_3 的SPI命令
  {
    _legController->updateCommand(tiBoardCommand);
  } 
  else                                              //既不是CHEETAH_3和不是MINI_CHEETAH就抛出错误
  {
    assert(false);
  }
  
  /*（2）设置腿部控制器和状态估计器的数据到LCM，再使用LCM发布出去*/
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);    //设置腿部控制器LCM数据
  _stateEstimate.setLcm(state_estimator_lcm);                                 //设置状态估计器LCM数据
  _lcm.publish("leg_control_command", &leg_control_command_lcm);              //LCM发布腿部控制器命令
  _lcm.publish("leg_control_data", &leg_control_data_lcm);                    //LCM发布腿部控制器数据
  _lcm.publish("state_estimator", &state_estimator_lcm);                      //LCM发布状态估计器状态
  _iterations++;
}


/*!
  * 功能：重置给定模式下的状态估计器
  * @param cheaterMode  检查模式
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) 
{
  _stateEstimator->removeAllEstimators();                   //（1）删除所有估计器
  _stateEstimator->addEstimator<ContactEstimator<float>>(); //（2）添加接触状态估计器
  Vec4<float> contactDefault;                               //（3）定义一个4维接触数组向量contactDefault
  contactDefault << 0.5, 0.5, 0.5, 0.5;                     //（4）定义四条腿的触地占空比
  _stateEstimator->setContactPhase(contactDefault);         //（5）使用contactDefault设置接触相序
  
  //（6）创建一系列子状态估计器
  if (cheaterMode)    //1）若在检查（仿真）模式下时，创建方向、位置、线速度状态估计器
  {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();        //（1）添加仿真方向估计器
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();   //（2）添加仿真位置和速度估计器
  } 
  else               //2）真实调试模式下时，创建方向、位置、线速度状态估计器
  {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();     //（1）添加真实IMU方向估计器，imu已经提供了方向,我们只需读取IMU数据并作相应的左边变换，返回那个IMU的值
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>(); //（2）添加真实基于卡尔曼滤波的位置和速度估计器
  }
}


void RobotRunner::cleanup() 
{}
