/*! @file ContactEstimator.cpp
 *  @brief 所有接触估计算法
 * 
 *此文件将包含所有接触检测算法。现在，只是
 *具有将相位估计传递到状态的传递算法
 *估计员。一旦我们将接触检测移动到C++，这将需要改变。
 */

#include "Controllers/ContactEstimator.h"

//现在这套程序没有腿部触地估计，是基于相位周期切换步态的