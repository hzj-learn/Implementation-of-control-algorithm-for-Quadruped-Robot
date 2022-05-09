/*============================ Vision =============================*/
/**
*机器人移动的FSM状态。管理特定于联系人的逻辑并处理调用控制器的接口。这种状态应该独立于控制器、步态和期望的轨迹。
 */

#include "FSM_State_Vision.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
  *功能：向传递状态特定信息的FSM状态的构造函数
  *通用FSM状态构造函数。
  * @param _controlFSMData  保存所有相关的控制数据
 */
template <typename T>
FSM_State_Vision<T>::FSM_State_Vision(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::VISION, "VISION"),
        vision_MPC(_controlFSMData->controlParameters->controller_dt,
                30 / (1000. * _controlFSMData->controlParameters->controller_dt),
                _controlFSMData->userParameters),
        cMPCOld(_controlFSMData->controlParameters->controller_dt,
                30 / (1000. * _controlFSMData->controlParameters->controller_dt),
                _controlFSMData->userParameters),
         _visionLCM(getLcmUrl(255))
{
  this->turnOnAllSafetyChecks();                  //设置安全检查
  this->checkPDesFoot = false;                    //关闭脚踏pos命令，因为它在WBC中设置为操作任务
  this->footFeedForwardForces = Mat34<T>::Zero(); //将GRF和足迹位置初始化为0
  this->footstepLocations = Mat34<T>::Zero();     //将足迹位置初始化为0
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());   //WBC控制
  _wbc_data = new LocomotionCtrlData<T>();                                        //WBC数据
  zero_vec3.setZero();                                                            //设置矩阵为0
  _global_robot_loc.setZero();                                                    //设置全局位置为0
  _robot_rpy.setZero();                                                           //设置rpy三个角为0

  _visionLCM.subscribe("local_heightmap", &FSM_State_Vision<T>::handleHeightmapLCM, this);    //LCM订阅local_heightmap
  _visionLCM.subscribe("traversability", &FSM_State_Vision<T>::handleIndexmapLCM, this);      //LCM订阅traversability
  _visionLCM.subscribe("global_to_robot", &FSM_State_Vision<T>::handleLocalization, this);    //LCM订阅global_to_robot
  _visionLCMThread = std::thread(&FSM_State_Vision<T>::visionLCMThread, this);                //从线程获取数据

  _height_map = DMat<T>::Zero(x_size, y_size);
  _idx_map = DMat<int>::Zero(x_size, y_size);
}



/**
  *功能：接收数据来设置_robot_rpy、_global_robot_loc
 */
template<typename T>
void FSM_State_Vision<T>::handleLocalization(const lcm::ReceiveBuffer* rbuf, 
    const std::string& chan, const localization_lcmt* msg)
{
  (void)rbuf;
  (void)chan;
  for(size_t i(0); i<3; ++i)
  {
    _robot_rpy[i] = msg->rpy[i];
    _global_robot_loc[i] = msg->xyz[i];
  }
  _b_localization_data = true;
}



template<typename T>
void FSM_State_Vision<T>::handleHeightmapLCM(const lcm::ReceiveBuffer *rbuf,
                                      const std::string &chan,
                                      const heightmap_t *msg)
{
  (void)rbuf;
  (void)chan;

  for(size_t i(0); i<x_size; ++i){
    for(size_t j(0); j<y_size; ++j){
      _height_map(i,j) = msg->map[i][j];
    }
  }
}


template<typename T>
void FSM_State_Vision<T>::handleIndexmapLCM(const lcm::ReceiveBuffer *rbuf,
    const std::string &chan,
    const traversability_map_t *msg) 
{
  (void)rbuf;
  (void)chan;

  for(size_t i(0); i<x_size; ++i){
    for(size_t j(0); j<y_size; ++j){
      _idx_map(i,j) = msg->map[i][j];
    }
  }
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_Vision<T>::onEnter() 
{
  this->nextStateName = this->stateName;  //默认为不转换
  this->transitionData.zero();            //重置转换数据
  vision_MPC.initialize();
  if(_b_localization_data)
  {
    _updateStateEstimator();
    _ini_body_pos = _global_robot_loc;
    _ini_body_ori_rpy = _robot_rpy;
  }
  else
  {
    _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;
    _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  }
  printf("[FSM VISION] On Enter\n");
}



/**
 * 功能：调用要在每个控制循环迭代中执行的函数
 */
template <typename T>
void FSM_State_Vision<T>::run() 
{
  if(_b_localization_data)
  {
    _updateStateEstimator();//（1）更新状态估计器数据
  }
/*调用此迭代的运动控制逻辑*/
  Vec3<T> des_vel; //des_vel是 x,y, yaw的三维向量
  _UpdateObstacle();        //（2）更新障碍位置数据、检查障碍位置和机器人位置的距离

 /*视觉行走*/
  _UpdateVelCommand(des_vel);       //（3）计算期望速度指令,通过设计腿部目标位置，用比例控制器求得期望速度指令
  _LocomotionControlStep(des_vel);  //（4）根据速度指令，计算每只脚的腿部控制器的命令
  // Convex Locomotion
  //_RCLocomotionControl();
  // Stand still
  //_JPosStand(); 
  _Visualization(des_vel);          //（5）在GUI进行可视化
}


/**
 * 功能：手柄实现运动控制
 */
template <typename T>
void FSM_State_Vision<T>::_RCLocomotionControl() 
{
  cMPCOld.run<T>(*this->_data); 
  if(this->_data->userParameters->use_wbc > 0.9)//使用遥控器
  {
    _wbc_data->pBody_des = cMPCOld.pBody_des;
    _wbc_data->vBody_des = cMPCOld.vBody_des;
    _wbc_data->aBody_des = cMPCOld.aBody_des;
    _wbc_data->pBody_RPY_des = cMPCOld.pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld.vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i)    //更新多部数据
    {
      _wbc_data->pFoot_des[i] = cMPCOld.pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld.vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld.aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld.Fr_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld.contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }
}


/**
 * 功能：更新状态估计器数据
 */
template<typename T>
void FSM_State_Vision<T>::_updateStateEstimator()
{
  StateEstimate<T> * estimate_handle = this->_data->_stateEstimator->getResultHandle();//中断获取状态估计器数据
  estimate_handle->position = _global_robot_loc;                                       //更新状态估计器中，机器人世界坐标系的位置
  estimate_handle->rpy = _robot_rpy;                                                   //更新状态估计器中，机器人rpy滚动、倾斜、偏航角
  estimate_handle->rBody = rpyToRotMat(_robot_rpy);                                    //更新状态估计器中，计算旋转矩阵，机器人rpy转换到旋转矩阵表达
  estimate_handle->orientation = rpyToQuat(_robot_rpy);                                //更新状态估计器中，计算四元素矩阵，机器人rpy转换到四元素矩阵表达
}


/**
 * 功能：使关节电机站起来
 */
template<typename T>
void FSM_State_Vision<T>::_JPosStand()
{
  Vec3<T> stand_jpos;
  stand_jpos << 0.f, -.8f, 1.6f;
  for(int leg(0); leg<4;++leg)
  {
    this->jointPDControl(leg, stand_jpos, zero_vec3);
  }
}


/**
 * 功能：更新障碍位置数据、检查障碍位置和机器人位置的距离
 */
template <typename T>
void FSM_State_Vision<T>::_UpdateObstacle()
{
  _obs_list.clear();         //障碍列表清空
  T obstacle_height = 0.15;  //障碍高度
  T threshold_gap = 0.08;    //阈值间隙
  bool add_obs(true);        //添加障碍
  //（1）更新机器人在世界坐标系的位置
  Vec3<T> robot_loc; 
  if(_b_localization_data)   //手动更新机器人世界坐标系的位置
  { 
    robot_loc = _global_robot_loc; 
  } 
  else                       //不手动更新机器人世界坐标系的位置，使用状态估计器估计的质心位置数据
  { 
    robot_loc = (this->_data->_stateEstimator->getResult()).position; 
  } 
  
  //（2）检查障碍位置和机器人位置的距离
  Vec3<T> obs; obs[2] = 0.27; 
  T dist = 0;                     //定义机器人位置和障碍物位置的距离
  for(size_t i(0); i<x_size; ++i)//可视化图形方块X尺寸， x loop
  {
    for(size_t j(0); j<y_size; ++j)//可视化图形方块Y尺寸， y loop
    {
      if( (_height_map(i, j) > obstacle_height) )// 机器人高度如果过障碍物的高度
      { 
        add_obs = true;
        obs[0] = i*grid_size - 50*grid_size + robot_loc[0];
        obs[1] = j*grid_size - 50*grid_size + robot_loc[1];

        for(size_t idx_obs(0); idx_obs < _obs_list.size(); ++idx_obs) //检查已添加的障碍物的距离
        {
          //计算机器人何障碍的距离，这是条空间两点距离公式
          dist = sqrt(  (obs[0] - _obs_list[idx_obs][0])*(obs[0] - _obs_list[idx_obs][0]) + 
              (obs[1] - _obs_list[idx_obs][1])*(obs[1] - _obs_list[idx_obs][1]) ); 
         
          if(dist < threshold_gap)//距离小于阈值间隙
          {
            add_obs = false;      //取消障碍
          }
        }

        if(add_obs)//添加的障碍物
        {
            _obs_list.push_back(obs);
        }
      }
    } 
  } 
}



template <typename T>
void FSM_State_Vision<T>::_print_obstacle_list()
{
  for(size_t i(0); i < _obs_list.size(); ++i)
  {
    printf("%lu th obstacle: %f, %f, %f\n", i, _obs_list[i][0], _obs_list[i][1], _obs_list[i][2] );
  }
}



template <typename T>
void FSM_State_Vision<T>::_Visualization(const Vec3<T> & des_vel)
{
  velocity_visual_t vel_visual;
  for(size_t i(0); i<3; ++i)
  {
    vel_visual.vel_cmd[i] = des_vel[i];
    vel_visual.base_position[i] = (this->_data->_stateEstimator->getResult()).position[i];
  }
  _visionLCM.publish("velocity_cmd", &vel_visual);
  _obs_visual_lcm.num_obs = _obs_list.size();
  for(size_t i(0); i<_obs_list.size(); ++i)
  {
    _obs_visual_lcm.location[i][0] = _obs_list[i][0];
    _obs_visual_lcm.location[i][1] = _obs_list[i][1];
    _obs_visual_lcm.location[i][2] = _obs_list[i][2];
  }
  _obs_visual_lcm.sigma = 0.15;
  _obs_visual_lcm.height = 0.5;
  _visionLCM.publish("obstacle_visual", &_obs_visual_lcm);
}


//功能：计算期望速度指令
//步骤：通过设计腿部目标位置，用比例控制器求得期望速度指令
template <typename T>
void FSM_State_Vision<T>::_UpdateVelCommand(Vec3<T> & des_vel) 
{
  des_vel.setZero();                            //形参，设置期望速度都为0
  Vec3<T> target_pos, curr_pos, curr_ori_rpy;   //定义目标位置、当前位置、当前rpy角度
  target_pos.setZero();                         //目标位置设置为0 
  T moving_time = 25.0;                         //移动时间
  T curr_time = (T)iter * 0.002;                //当前时间
  target_pos[0] = 1.0 * (1-cos(2*M_PI*curr_time/moving_time));    //这条目标位置公式实质就是余弦函数，target_pos=1-cos(360-相位变换)

  if(_b_localization_data)          //手动设定机器人位置、角度数据，通过手动赋值给数据
  {
    //计算当前位置
    curr_pos = _global_robot_loc;
    curr_pos -= _ini_body_pos;
    //计算rpy角度
    curr_ori_rpy = _robot_rpy;
  }
  else                              //自动设定机器人位置、角度数据，通过状态估计器给数据
  {
    //计算当前位置
    curr_pos = (this->_data->_stateEstimator->getResult()).position;
    curr_pos -= _ini_body_pos;
    //计算rpy角度
    curr_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  }
  target_pos = rpyToRotMat(_ini_body_ori_rpy).transpose() * target_pos;
  des_vel[0] = 0.7 * (target_pos[0] - curr_pos[0]);             //躯干前进线速度，类似于比例控制
  des_vel[1] = 0.7 * (target_pos[1] - curr_pos[1]);             //躯干横向线速度，类似于比例控制
  des_vel[2] = 0.7 * (_ini_body_ori_rpy[2] - curr_ori_rpy[2]);  //躯干航向速度  ，类似于比例控制

  T inerproduct;
  T x, y, x_obs, y_obs;
  T vel_x, vel_y;
  T sigma(0.15);
  T h(0.5);                                   //高度
  for(size_t i(0); i<_obs_list.size(); ++i)
  {
    x = curr_pos[0];
    y = curr_pos[1];
    x_obs = _obs_list[i][0];
    y_obs = _obs_list[i][1];
    h = _obs_list[i][2];

    inerproduct = (x-x_obs)*(x-x_obs) + (y-y_obs)*(y-y_obs);
    vel_x = h*((x-x_obs)/ sigma/sigma)*exp(-inerproduct/(2*sigma*sigma));
    vel_y = h*((y-y_obs)/ sigma/sigma)*exp(-inerproduct/(2*sigma*sigma));
    //printf("vel_x, y: %f, %f\n", vel_x, vel_y);
    des_vel[0] += vel_x;
    des_vel[1] += vel_y;
  }
  //printf("des vel_x, y: %f, %f\n", des_vel[0], des_vel[1]);
  //期望速度限幅
  des_vel[0] = fminf(fmaxf(des_vel[0], -1.), 1.);
  des_vel[1] = fminf(fmaxf(des_vel[1], -1.), 1.);
}
 


/**
 *管理用户可以转换为哪种状态
 *命令或状态事件触发器。
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_Vision<T>::checkTransition() 
{
  iter++;//获取下一个状态
//切换FSM控制模式
  switch ((int)this->_data->controlParameters->control_mode) 
  {
    case K_VISION:
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->transitionDuration = 0.0;
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;//要求更改平衡表
      this->transitionDuration = 0.0;//过渡时间很快
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;//要求更改平衡表
      this->transitionDuration = 0.0;//过渡时间很快
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_VISION << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;//将下一个状态名返回给FSM
}


/**
 *处理robot在状态之间的实际转换。
 *转换完成时返回true。
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_Vision<T>::transition() 
{
//切换FSM控制模式
  switch (this->nextStateName) 
  {
    case FSM_StateName::BALANCE_STAND:
      _LocomotionControlStep(zero_vec3);
      iter++;
      if (iter >= this->transitionDuration * 1000) 
      {
        this->transitionData.done = true;
      }
       else 
      {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;//将转换数据返回到FSM
}


/**
*在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_Vision<T>::onExit() 
{
//退出时无需清理
  iter = 0;
}




/**
 *功能：计算每只脚的腿部控制器的命令
 *调用适当的平衡控制器并分析结果
 *每个姿势或摆动腿。
 */
template <typename T>
void FSM_State_Vision<T>::_LocomotionControlStep(const Vec3<T> & des_vel) {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

//接触状态逻辑
  vision_MPC.run<T>(*this->_data, des_vel, _height_map, _idx_map);
  if(this->_data->userParameters->use_wbc > 0.9)
  {
    _wbc_data->pBody_des = vision_MPC.pBody_des;
    _wbc_data->vBody_des = vision_MPC.vBody_des;
    _wbc_data->aBody_des = vision_MPC.aBody_des;

    _wbc_data->pBody_RPY_des = vision_MPC.pBody_RPY_des;
    _wbc_data->vBody_Ori_des = vision_MPC.vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i)
    {
      _wbc_data->pFoot_des[i] = vision_MPC.pFoot_des[i];
      _wbc_data->vFoot_des[i] = vision_MPC.vFoot_des[i];
      _wbc_data->aFoot_des[i] = vision_MPC.aFoot_des[i];
      _wbc_data->Fr_des[i] = vision_MPC.Fr_des[i]; 
    }
    _wbc_data->contact_state = vision_MPC.contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }
}

template class FSM_State_Vision<float>;
