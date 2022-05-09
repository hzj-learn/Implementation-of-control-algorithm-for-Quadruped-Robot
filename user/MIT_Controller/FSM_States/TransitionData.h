/*============================= FSM State =============================*/
#ifndef TRANSITIONDATA_H
#define TRANSITIONDATA_H

/**
 * 相关数据的结构，可在转换期间用于在状态间传递数据。
 */
template <typename T>

//切换数据的结构体
struct TransitionData 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransitionData() { zero(); }
  void zero()            // 把所有数据清零
  {
    done = false;     //在转换完成时标记
    t0 = 0.0;         //转变开始的时间
    tCurrent = 0.0;   //当前时间自过渡开始
    tDuration = 0.0;  //整体的过渡时间

    //机器人在过渡开始时的状态
    comState0 = Vec12<T>::Zero();  // 质心状态
    qJoints0 = Vec12<T>::Zero();   // 关节位置
    pFoot0 = Mat34<T>::Zero();     // 足端位置

    //当前机器人状态
    comState = Vec12<T>::Zero();  //质心状态
    qJoints = Vec12<T>::Zero();   //关节位置
    pFoot = Mat34<T>::Zero();     //足端位置
  }
  bool done = false;     //在转换完成时标记
  /*时间参数*/
  T t0;         //转变开始的时间
  T tCurrent;   //当前时间自过渡开始
  T tDuration;  //整体的过渡时间

  /*机器人在过渡开始时的状态*/
  Vec12<T> comState0;  //质心状态
  Vec12<T> qJoints0;   //关节位置
  Mat34<T> pFoot0;     //足端位置

  /*当前机器人状态*/
  Vec12<T> comState;  //质心状态
  Vec12<T> qJoints;   //关节位置
  Mat34<T> pFoot;     //足端位置
};

template struct TransitionData<double>;
template struct TransitionData<float>;

#endif  // CONTROLFSM_H