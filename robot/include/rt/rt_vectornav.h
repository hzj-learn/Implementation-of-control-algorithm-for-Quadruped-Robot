/*!
 * @file rt_vectornav.h
 * @brief 矢量导航IMU通信
 */

#ifndef _rt_vectornav
#define _rt_vectornav

#ifdef linux

#include <lcm/lcm-cpp.hpp>
#include "SimUtilities/IMUTypes.h"

#ifdef __cplusplus
extern "C" 
{
#endif

#include "vn/sensors.h"

#ifdef __cplusplus
}
#endif

bool init_vectornav(VectorNavData* vd_data);

#endif
#endif
