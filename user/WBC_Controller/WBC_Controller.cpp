#include "WBC_Controller.hpp"
#include "WBC_States/Cheetah_DynaCtrl_Definition.h"

#include <WBC_States/BackFlip/BackFlipTest.hpp>
#include <WBC_States/BodyCtrl/BodyCtrlTest.hpp>
#include <WBC_States/Bounding/BoundingTest.hpp>
#include <WBC_States/JPosCtrl/JPosCtrlTest.hpp>
#include <WBC_States/WBICTrot/WBICTrotTest.hpp>

#include <ParamHandler.hpp>
#include <Utilities/Timer.h>
#include <unistd.h>

#include "rt/rt_interface_lcm.h"

#define REMOTE_CTRL false
//#define REMOTE_CTRL true

WBC_Controller::WBC_Controller():RobotController()
{
  _data = new Cheetah_Data<float>();
  _extra_data = new Cheetah_Extra_Data<float>();
}


/**
 * 功能：初始化WBC函数
 */
void WBC_Controller::initializeController() 
{
  //（1）从参数文件中获取请求的测试名称
  ParamHandler handler(THIS_COM
                       "user/WBC_Controller/WBC_States/config/ROBOT_test_setup.yaml");
  std::string test_name;
  if (!handler.getString("test_name", test_name)) //若找不到测试名称，报错退出
  {
    printf("cannot find test name\n");
    exit(0);
  }

  //（2）从测试名称字符串运行请求的测试
  if (test_name == "body_ctrl") 
  {
    _wbc_state = new BodyCtrlTest<float>(_model, _robotType);
  } 
  else if (test_name == "wbic_trot") 
  {
    _wbc_state = new WBICTrotTest<float>(_model, _robotType);
  } 
  else if (test_name == "bounding") 
  {
    _wbc_state = new BoundingTest<float>(_model, _robotType);
  } 
  else if (test_name == "back_flip") 
  {
    _wbc_state = new BackFlipTest<float>(_model, _robotType);
  }
}


/**
 * 功能：使用WBC逻辑计算leg控制器的命令。
 */
void WBC_Controller::runController() 
{
 ///////////*（1）判断有没有使用手柄，若有优先使用手柄的数据，若没有使用GUI设置的控制数据*////////////////////
#if (REMOTE_CTRL)//默认没有使用手柄
  get_main_control_settings(&main_control_settings);
  // printf("%f\n", main_control_settings.mode);
#else
  main_control_settings.mode = 11;
#endif

 ///////////*（2）判断机器人是否在停止状态*//////////////////////////////////////////////////////////////
 /* 若是在E-Stop状态*/
  if (main_control_settings.mode == 0) 
  {  
    for (int leg = 0; leg < 4; leg++) 
    {
      for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) 
      {
        _legController->commands[leg].tauFeedForward[jidx] = 0.;    //1）每个关节前馈力矩设置为0
        _legController->commands[leg].qDes[jidx] =                  //2）每个关节依次设置关节角度
            _legController->datas[leg].q[jidx];
        _legController->commands[leg].qdDes[jidx] = 0.;             //3）每个关节角速度设置为0
      }
      _legController->commands[leg].kpJoint.setZero();              //4）每个关节KP反馈增益设置为0
      _legController->commands[leg].kdJoint.setZero();              //5）每个关节KD反馈增益设置为0
    }
  } 

  /*若不是E-Stop状态，正常运行WBC*/
  else                                 
  {
      static bool b_first_visit(true);
      ///////////////////////////////*(1)若是第一次运行，先从状态估计器中取偏航角的值，作为偏航角的初始值*/////////////
    if(b_first_visit)        
      {
        _ini_yaw = _stateEstimator->getResult().rpy[2];
        b_first_visit = false;
      }

    ////////////////////////////////*(2)WBC状态命令计算*//////////////////////////////////////////////////////////
    // 对WBC进行运动控制测试的评价
    Mat3<float> kpMat;                                  //（1）定义kp反馈增益
    Mat3<float> kdMat;                                  //（2）定义kd反馈增益
    Vec3<float> rpy = _stateEstimator->getResult().rpy; //（3）通过状态估计器的rpy数据，定义并赋值躯干rpy
    rpy[2] -= _ini_yaw;                                 //（4）计算当前的偏航角rpy[2]，公式：当前的偏航角=状态估计器的rpy[2]的值-偏航角初始化的值
    Quat<float> quat_ori = ori::rpyToQuat(rpy);         //（5）把躯干的rpy转换为四元素

    for (size_t i(0); i < 4; ++i)                       //（6）通过四元数进行躯干方向设置（四元数有四个元素）
    {
        _data->body_ori[i] = quat_ori[i];
    }
    
    for (int i(0); i < 3; ++i)                          //（7）通过状态估计器的欧拉角进行躯干角速度设置（欧拉角有三个元素）
    {
        _data->ang_vel[i] = _stateEstimator->getResult().omegaBody[i];
    }

    for (int leg(0); leg < 4; ++leg)                    //（8）设置12个关节的角度和角速度数据，此时数据还没有传到腿部控制器
    {
        for (int jidx(0); jidx < 3; ++jidx) 
        {
            _data->jpos[3 * leg + jidx] = _legController->datas[leg].q[jidx];
            _data->jvel[3 * leg + jidx] = _legController->datas[leg].qd[jidx];
        }
    }
    _data->mode = main_control_settings.mode;           //（9）设置控制模式

    //////////////////////////////*（3）从手柄数据中设置参数*/////////////////////////////////////////////////////////
    //手柄方向驱动指令设置
    _data->dir_command[0] = _driverCommand->leftStickAnalog[1];
    _data->dir_command[1] = _driverCommand->leftStickAnalog[0];

    //手柄方向驱动指令设置
    _data->ori_command[0] = _driverCommand->rightTriggerAnalog;
    _data->ori_command[0] -= _driverCommand->leftTriggerAnalog;

   //手柄方向驱动指令设置
    _data->ori_command[1] = _driverCommand->rightStickAnalog[1];
    _data->ori_command[2] = _driverCommand->rightStickAnalog[0];

///////////////////////////*(4)若有打开移动控制*/////////////////////////////////////////////////////////////////////
#if (REMOTE_CTRL)
    // Remote control
    if (main_control_settings.variable[0] == 4) // 若处于躯干姿态控制，更新计算欧拉角rpy
    {  
        _data->ori_command[0] = main_control_settings.rpy_des[0];
        _data->ori_command[1] = main_control_settings.rpy_des[1];
        _data->ori_command[2] = main_control_settings.rpy_des[2];
    } 
    else                                        //若处于躯干平移控制，计算躯干方向速度
    {
        _data->dir_command[0] = main_control_settings.v_des[0];
        _data->dir_command[1] = main_control_settings.v_des[1];
        _data->ori_command[2] = main_control_settings.omega_des[2];
    }
#endif

/////////////////////////*（5）【重要！！】WBC获取腿部指令和外部数据*///////////////////////////////////////////////////
    // Timer timer;//计时器计时
    _wbc_state->GetCommand(_data, _legController->commands, _extra_data);
    // std::cout<< "wbc computation: " << timer.getMs()<<std::endl;
    // === End of WBC state command computation  =========== //
  }
}


/**
 * 功能：更新可视化
 */
void WBC_Controller::updateVisualization()
{
    _StepLocationVisualization();
    _BodyPathVisualization();
    _BodyPathArrowVisualization();
    _testDebugVisualization();
}


/**
 * 功能：落脚可视化
 */
void WBC_Controller::_StepLocationVisualization() 
{
    // 椎体
    // _visualizationData->num_cones = 20*2;
    int num_step = _extra_data->num_step;
    _visualizationData->num_cones = num_step;
    for (int j(0); j < num_step; j++) 
    {
        ConeVisualization cone;
        cone.radius = 0.03;
        cone.direction << 0, 0, 0.05;
        cone.point_position << _extra_data->loc_x[j], _extra_data->loc_y[j],
        (_extra_data->loc_z[j] - 0.5);  // Ground is -0.5
        cone.color << .6, .2, .4, .6;
        _visualizationData->cones[j] = cone;
    }
}

/**
 * 功能：躯干路径可视化
 */
void WBC_Controller::_BodyPathVisualization() 
{
    // 路径可视化
    PathVisualization path;
    path.num_points = _extra_data->num_path_pt;
    for (size_t j = 0; j < path.num_points; j++) 
    {
        path.position[j] << _extra_data->path_x[j], _extra_data->path_y[j],
        _extra_data->path_z[j] - 0.5;  // Ground is -0.5
    }
    path.color << 0.6, 0.2, 0.05, 1;

    _visualizationData->num_paths = 1;
    _visualizationData->paths[0] = path;
}

/**
 * 功能：躯干路径方向可视化
 */
void WBC_Controller::_BodyPathArrowVisualization() 
{
    _visualizationData->num_arrows = _extra_data->num_middle_pt;

    double ar_len(0.07);
    double yaw;
    for (int i(0); i < _extra_data->num_middle_pt; ++i) 
    {
    _visualizationData->arrows[i].base_position << _extra_data->mid_x[i],
    _extra_data->mid_y[i], _extra_data->mid_z[i] - 0.5;

    yaw = _extra_data->mid_ori_yaw[i];

    _visualizationData->arrows[i].direction << ar_len * cos(yaw),
        ar_len * (sin(yaw)), 0.;

    _visualizationData->arrows[i].head_width = 0.02;
    _visualizationData->arrows[i].head_length = 0.03;
    _visualizationData->arrows[i].shaft_width = 0.01;

    _visualizationData->arrows[i].color << 0.8, 0.3, 0.1, 1;
  }
}

/**
 * 功能：测试调试可视化
 */
void WBC_Controller::_testDebugVisualization() 
{
  // Todo：将其中的一部分移到机器人控制器中（ robot controller）
    static long long _iterations(0);
    ++_iterations;
  float t = (float)_iterations / 1000;
  return;

  //测试球可视化
  SphereVisualization sphere;
  sphere.position[0] = 0;
  sphere.position[1] = 5 * std::sin(t);
  sphere.position[2] = 5 * std::cos(t);
  sphere.radius = 1;
  sphere.color[0] = 1;
  sphere.color[1] = 1;
  sphere.color[2] = 0;
  sphere.color[3] = 1;
  _visualizationData->num_spheres = 1;
  _visualizationData->spheres[0] = sphere;

  //椎体
    _visualizationData->num_cones = 5;
  for (size_t j = 0; j < 5; j++) 
  {
    ConeVisualization cone;
    cone.radius = (j + 1) / 10.;
    cone.direction << 0, 0, .3 * j + .2;
    cone.direction += .2f * j *
                      Vec3<float>(sinf(.4f * j * t + j * .2f),
                                  cosf(.4f * j * t + j * .2f), 0);
    cone.point_position << 3 + j, 3, 0;
    cone.color << .4, .1, .2, .6;
    _visualizationData->cones[j] = cone;
  }

  // boxes
  _visualizationData->num_blocks = 1;
  BlockVisualization block;
  block.corner_position << -5, -5, 0;
  block.dimension << 1, .2, .3;
  block.color << 1, 0, 0, std::fmod((float)_iterations / 1000, 1.f);
  _visualizationData->blocks[0] = block;

  // 测试方向可视化
  _visualizationData->num_arrows = 1;

  _visualizationData->arrows[0].base_position << 1, 1, 1;

  _visualizationData->arrows[0].direction << 1, 1, 1;

  _visualizationData->arrows[0].head_width = 0.1;
  _visualizationData->arrows[0].head_length = 0.2;
  _visualizationData->arrows[0].shaft_width = 0.05;

  _visualizationData->arrows[0].color
      << std::fmod((float)_iterations / 1000 + 0.5f, 1.f),
      std::fmod((float)_iterations / 1000, 1.f), 1, .6;

  //测试路径可视化
  PathVisualization path;
  path.num_points = 150;
  for (size_t j = 0; j < path.num_points; j++) 
  {
    path.position[j] << j * 2.0 / path.num_points,
        sin(j * 10. / path.num_points + ((float)_iterations) / 1000), .5;
  }
  path.color << 0, 0, 1, 1;

  _visualizationData->num_paths = 1;
  _visualizationData->paths[0] = path;
}


