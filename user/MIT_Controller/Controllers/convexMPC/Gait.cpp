#include "Gait.h"

///////////////
// 原理：
// GAIT  参考Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains·（1）
// 和 Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control·（2）
// 一个步态周期由horizonLength(10)个mpc周期组成 
// 步态按1KHz处理 mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms
// 则步态周期为 10*30 =300ms 
// 一个步态周期被分为horizonLength(10)段 

// offsets是步态的相关相位参数，durations支撑时间参数，用整数即分段来计算
// _offsetsFloat、_durationsFloat在0~1之间
///////////////


/**
 * 功能：偏移、连续的步态构造函数，按分段算
 */
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),               //相位差
  _durations(durations.array()),           //支撑时间
  _nIterations(nSegment)                   //步态周期分段数
{
  _name = name;
  _mpc_table = new int[nSegment * 4];                           //为MPC步态表分配内存
  _offsetsFloat = offsets.cast<float>() / (float) nSegment;     //对应步态的腿之间相位差
  _durationsFloat = durations.cast<float>() / (float) nSegment; //对应步态支撑持续时间在整个周期百分比
  _stance = durations[0];                                       //支撑持续时间 用整个过程的分段计数
  _swing = nSegment - durations[0];                             //摆动持续时间 同上
}


/**
 * 功能：偏移、连续的步态析构函数
 */
OffsetDurationGait::~OffsetDurationGait() 
{
  delete[] _mpc_table;
}


/**
 * 功能：支撑步态构造函数，按百分比算
 */
MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string &name) 
{
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

/**
 * 功能：混合频率步态析构函数
 */
MixedFrequncyGait::~MixedFrequncyGait() 
{
  delete[] _mpc_table;
}


/**
 * 功能：摆动腿判断获取接触状态
 */
Vec4<float> OffsetDurationGait::getContactState() 
{
  Array4f progress = _phase - _offsetsFloat; //progress每条腿在整个步态周期的位置 offest是相位差补偿

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.; 
    if(progress[i] > _durationsFloat[i]) //相位大于支撑结束相位，非支撑状态
    {
      progress[i] = 0.;
    }
    else                                 //相位小于支撑结束相位，支撑状态
    {
      progress[i] = progress[i] / _durationsFloat[i]; //相位在支撑相中的百分比
    }
  }
  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

/**
 * 功能：支撑腿获取接触状态
 */
Vec4<float> MixedFrequncyGait::getContactState() 
{
  Array4f progress = _phase;
  for(int i = 0; i < 4; i++) 
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _duty_cycle) 
    {
      progress[i] = 0.;
    } 
    else 
    {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


/**
 * 功能：获取摆动状态
 */
Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;
  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.; //相位大于摆动结束相位，非摆动状态
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];//相位在摆动相中的百分比
    } 
  }
  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


/**
 * 功能：获取摆动状态
 */
Vec4<float> MixedFrequncyGait::getSwingState() 
{
  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for(int i = 0; i < 4; i++) 
  {
    if(progress[i] < 0) 
    {
      progress[i] = 0;
    } 
    else 
    {
      progress[i] = progress[i] / swing_duration;
    }
  }
  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


/**
 * 功能：为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息
 */
int* OffsetDurationGait::getMpcTable()
{
  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
	  
      if(progress[j] < _durations[j]) //在接触时间内 
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }



  return _mpc_table;
}



/**
 * 功能：为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息
 */
int* MixedFrequncyGait::getMpcTable() 
{
  //printf("MPC table (%d):\n", _iteration);
  for(int i = 0; i < _nIterations; i++) 
  {
    for(int j = 0; j < 4; j++) 
    {
      int progress = (i + _iteration + 1) % _periods[j];  // progress
      if(progress < (_periods[j] * _duty_cycle)) 
      {
        _mpc_table[i*4 + j] = 1;
      } 
      else 
      {
        _mpc_table[i*4 + j] = 0;
      }
      //printf("%d %d (%d %d) | ", _mpc_table[i*4 + j], progress, _periods[j], (int)(_periods[j] * _duty_cycle));
    }
    //printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", _mpc_table[i*4], _mpc_table[i*4 + 1], _mpc_table[i*4 + ])
    //printf("\n");
  }
  return _mpc_table;
}


/**
 * 功能：设置MPC迭代的步态分段
 */
void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{	
  //细分为 nMPC_segments（10）个时间步 参考（2） A.Experimental Setup，当前在第几个步态分段中 0~9
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  //当前在整个步态周期百分比 一个步态周期为 nMPC_segments（10） 个mpc周期
  //               当前为站立			重复计数用									整个周期长度
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
}



/**
 * 功能：设置MPC迭代的步态分段
 */
void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) 
{
  _iteration = (currentIteration / iterationsBetweenMPC);//相当于现在步态相位时间在正MPC控制周期的占空比
  for(int i = 0; i < 4; i++) 
  {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
  }
  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);
}



/**
 * 功能：获取当前步态分段相位，摆动时用
 */
int OffsetDurationGait::getCurrentGaitPhase() 
{
  return _iteration;
}


/**
 * 功能：获取当前步态相位，摆动时用
 */
int MixedFrequncyGait::getCurrentGaitPhase() 
{
  return 0;
}


/**
 * 功能：获得摆动持续时间长度
 */
float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) 
{
  (void)leg;
  return dtMPC * _swing;
}


/**
 * 功能：获得摆动持续时间长度
 */
float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) 
{
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}


/**
 * 功能：获得站立持续时间长度
 */
float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) 
{
  (void) leg;
  return dtMPC * _stance;
}



/**
 * 功能：获得站立持续时间长度
 */
float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) 
{
  return dtMPC * _duty_cycle * _periods[leg];
}



void OffsetDurationGait::debugPrint() 
{
}

void MixedFrequncyGait::debugPrint() 
{
}