/*! @file ContactEstimator.h
 *  @brief 所有接触估计算法
 * 此文件将包含所有接触检测算法。目前，它只有一个传递算法，将相位估计传递给状态估计器。一旦我们将接触检测移动到C++，这将需要改变。
 *  我们还需要为“阶段”和“接触”建立公约。
 */

#ifndef PROJECT_CONTACTESTIMATOR_H
#define PROJECT_CONTACTESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * 返回预期接触状态的“passthrough”接触估计器
 */
template <typename T>
class ContactEstimator : public GenericEstimator<T> 
{
 public:
  /*!
   * 通过将已检测的腿部接触状态复制到估计接触状态
   * 备注：其实没有做触地检测，仅仅通过相位反馈的周期信息作为触地信号
   */
  virtual void run() 
  {
    this->_stateEstimatorData.result->contactEstimate =
        *this->_stateEstimatorData.contactPhase;
  }

  /*!
   * 设置接触估计器
   */
  virtual void setup() {}
};

#endif  // PROJECT_CONTACTESTIMATOR_H
