/*! @file FirstOrderIIRFilter.h
 *  @brief 一个简单的一阶滤波器
 *
 *
 */

#ifndef PROJECT_FIRSTORDERIIRFILTER_H
#define PROJECT_FIRSTORDERIIRFILTER_H

#include <cmath>

/*!
 * 一阶滤波器
 * @tparam T : 要筛选的数据类型
 * @tparam T2: 截止/采样频率、增益的浮点类型
 */
template <typename T, typename T2>
class FirstOrderIIRFilter 
{
 public:
  /*!
   * 创建一个简单的一阶滤波器
   * @param cutoffFrequency : 滤波器截止频率
   * @param sampleFrequency : 采样频率（调用更新的速率）
   * @param initialialValue : 存储在筛选器中的初始值
   */
  FirstOrderIIRFilter(T2 cutoffFrequency, T2 sampleFrequency, T& initialValue) 
  {
    _alpha = 1 - std::exp(-2 * M_PI * cutoffFrequency / sampleFrequency);
    _state = initialValue;
  }

  /*!
   * 创建一个简单的一阶滤波器
   * @param alpha :         过滤参数
   * @param initialValue :  初始值
   */
  FirstOrderIIRFilter(T2 alpha, T& initialValue)
      : _state(initialValue), _alpha(alpha) {}

  /*!
   * 使用新示例更新筛选器
   * @param x : 新样品
   * @return    过滤器的新状态
   */
  T update(T& x) 
  {
    _state = _alpha * x + (T2(1) - _alpha) * _state;
    return _state;
  }

  /*!
   * 获取筛选器的值，而不更新
   */
  T get() { return _state; }

  void reset() { _state *= T2(0); }

 private:
  T _state;
  T2 _alpha;
};

#endif  // PROJECT_FIRSTORDERIIRFILTER_H
