#ifndef CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include "cppTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;


class VisionGait
{
public:
  VisionGait(int nMPC_segments, Vec4<int> offsets, Vec4<int>  durations, const std::string& name="");
  ~VisionGait();
  Vec4<float> getContactState();        //获得四足接触状态
  Vec4<float> getSwingState();          //获得四足摆动状态
  int* mpc_gait();                      //获得mpc需要的gait数组 悬空与否
  void setIterations(int iterationsPerMPC, int currentIteration);
  int _stance;                          //支撑时间，按分段算
  int _swing;

private:
  int _nMPC_segments;
  int* _mpc_table;                      //储存每条腿是否悬空
  Array4i _offsets;                     //分段中的偏移量
  Array4i _durations;                   //分段中step的持续时间
  Array4f _offsetsFloat;                //腿之间相位补偿(0至1)
  Array4f _durationsFloat;              //相位持续时间(0到1)
  int _iteration;                       //步态片段计数
  int _nIterations;                     //步态片段数
  float _phase;                         //当前相位在整步态周期中
};


class VisionMPCLocomotion 
{
public:
  VisionMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
  void initialize();

  template<typename T>
  void run(ControlFSMData<T>& data, 
      const Vec3<T> & vel_cmd, const DMat<T> & height_map, const DMat<int> & idx_map);

  Vec3<float> pBody_des;    //期望
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];//足端期望 足端轨迹跟踪用
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,
      const DMat<float> & height_map, const DMat<int> & idx_map);
  void _IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected, 
      const DMat<int> & idx_map);

  Vec3<float> _fin_foot_loc[4];
  float grid_size = 0.015;      //height_map网格分辨率 
  Vec3<float> v_des_world;
  Vec3<float> rpy_des;
  Vec3<float> v_rpy_des;

  float _body_height = 0.31;    //机身高度
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
  int iterationsBetweenMPC;     //控制mpc频率用
  int horizonLength;            //mpc分段数，即预测多少个mpc周期长的输入
  float dt;                     //一般频率下时间间隔
  float dtMPC;                  //mpc运算周期
  int iterationCounter = 0;     //频率控制计数器
  Vec3<float> f_ff[4];          //四脚力输出
  Vec4<float> swingTimes;       //摆动时间 
  FootSwingTrajectory<float> footSwingTrajectories[4];                      //四条腿轨迹对象
  VisionGait trotting, bounding, pronking, galloping, standing, trotRunning;//调用构造函数时调用VisionGait构造函数实例化
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;                                 //pd控制参数
  bool firstRun = true;         //首次运行
  bool firstSwing[4];           //首次摆动
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;             //当前步态
  int gaitNumber;               //步态

  Vec3<float> world_position_desired;//机体期望位置
  Vec3<float> rpy_int;               //初始欧拉角
  Vec3<float> rpy_comp;
  Vec3<float> pFoot[4];              //四足端坐标
  float trajAll[12*36];              //mpc格式储存轨迹

  MIT_UserParameters* _parameters = nullptr;
};


#endif //CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
