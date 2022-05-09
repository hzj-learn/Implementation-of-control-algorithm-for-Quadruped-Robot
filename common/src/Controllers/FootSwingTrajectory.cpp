#include <include/Math/Interpolation.h>
#include "../../include/Controllers/FootSwingTrajectory.h"

/*!
 * 用bezier曲线计算脚的摆动轨迹
 * @tparam T
 * @param phase
 * @param swingTime
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) 
{
  _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);                                          //y0和yf之间的线性插值。x在0和1之间在两个值之间插值  
  _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;               //y0和yf之间的三次bezier插值导数。x在0和1之间
  _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);//y0和yf之间的三次bezier插值导数。x在0到之间1

  T zp, zv, za;
  if(phase < T(0.5)) //相位小于0.5
  {
    zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } 
  else              //相位大于0.5
  {
    zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;