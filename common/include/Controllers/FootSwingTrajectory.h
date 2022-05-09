/*!
 * @file FootSwingTrajectory.h
 * @brief 实用工具，以产生脚摆动轨迹。
 * 目前使用贝兹曲线，像猎豹3号
 */

#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"

/*!
 * 功能：单脚的摆动轨迹函数
 */
template<typename T>
class FootSwingTrajectory 
{
public:
  /*!
   * 功能：创建一个新的脚摆动轨迹，一切设置为零函数
   */
  FootSwingTrajectory() 
  {
    _p0.setZero();    //初始点
    _pf.setZero();    //终点
    _p.setZero();     //轨迹点
    _v.setZero();     //轨迹速度
    _a.setZero();     //轨迹加速度
    _height = 0;      //轨迹高度
  }

  /*!
   * 功能：设置脚的起点位置函数
   * @param p0 : 脚的初始位置
   */
  void setInitialPosition(Vec3<T> p0) 
  {
    _p0 = p0;
  }

  /*!
   * 功能：设置脚的终点位置函数
   * @param pf :最后的脚姿势
   */
  void setFinalPosition(Vec3<T> pf) 
  {
    _pf = pf;
  }

  /*!
   * 功能：设置摆动腿的最大高度函数
   * @param h :摆动腿的最大高度，在摆动腿进行到一半时达到
   */
  void setHeight(T h) 
  {
    _height = h;
  }

/*!
 * 功能：用bezier曲线计算脚的摆动轨迹
 * @tparam T
 * @param phase
 * @param swingTime
 */
  void computeSwingTrajectoryBezier(T phase, T swingTime);

  /*!
   * 功能：获得轨迹坐标，获得摆动腿的当前点位置的函数
   * @return :脚的位置
   */
  Vec3<T> getPosition() 
  {
    return _p;
  }

  /*!
   * 功能：获得此时轨迹导数，得到摆动腿上当前的脚速度函数
   * @return : 足部速度
   */
  Vec3<T> getVelocity() 
  {
    return _v;
  }

  /*!
   * 功能： 获取此时轨迹二次导数，得到脚在当前点上的加速函数
   * @return : 脚加速度
   */
  Vec3<T> getAcceleration() 
  {
    return _a;
  }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;
  T _height;
};


#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
