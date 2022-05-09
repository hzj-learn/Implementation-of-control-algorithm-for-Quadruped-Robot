/*!
 * @file rt_vectornav.cpp
 * @brief 矢量导航IMU通信
 */

//#ifdef linux

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <string>
#include <stdexcept>
#include <lcm/lcm-cpp.hpp>
#include "SimUtilities/IMUTypes.h"
#include "Utilities/utilities.h"
#include "rt/rt_vectornav.h"
#include "vectornav_lcmt.hpp"

#define K_MINI_CHEETAH_VECTOR_NAV_SERIAL "/dev/ttyS0"


int processErrorReceived(const std::string& errorMessage, VnError errorCode);
void vectornav_handler(void* userData, VnUartPacket* packet,
                       size_t running_index);


/*!
 * 功能：矢量导航驱动器数据的结构体
 */
typedef struct 
{
  VnSensor vs;
  BinaryOutputRegister bor;
} vn_sensor;

vn_sensor vn;
static lcm::LCM* vectornav_lcm;
vectornav_lcmt vectornav_lcm_data;
static VectorNavData* g_vn_data = nullptr;



/*!
 * 功能：初始化Vectornav矢量导航通信并设置传感器函数
 */
bool init_vectornav(VectorNavData* vn_data) 
{
  g_vn_data = vn_data;
  printf("[Simulation] Setup LCM...\n");

  //（1）IMU的LCM通讯进行初始化，分配一个LCM发布消息的内存给IMU
  vectornav_lcm = new lcm::LCM(getLcmUrl(255));

  //（2）若矢量导航LCM通讯没初始化，报错
  if (!vectornav_lcm->good())     
  {
    printf("[ERROR] Failed to set up LCM\n");
    throw std::runtime_error("lcm bad");
  }

  VnError error;
  VpeBasicControlRegister vpeReg;
  ImuFilteringConfigurationRegister filtReg;
  const char SENSOR_PORT[] = K_MINI_CHEETAH_VECTOR_NAV_SERIAL;    //定义传感器链接的port端口
  const uint32_t SENSOR_BAUDRATE = 115200;                        //定义通讯的波特率是115200
  char modelNumber[30];
  char strConversions[50];
  uint32_t newHz, oldHz;
  // uint32_t hz_desired = 200;
  printf("[rt_vectornav] init_vectornav()\n");

  //（3）初始化vectornav库
  VnSensor_initialize(&(vn.vs));//这个函数是传感器厂家提供的底层驱动函数，我们调用就行，不用具体管的

  //（4）若没有连接到IMU传感器，报错 
  if ((error = VnSensor_connect(&(vn.vs), SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_connect failed.\n");
    processErrorReceived("Error connecting to sensor.", error);
    return false;
  }

  //（5）若没有读取传感器型号 ，报错
  if ((error = VnSensor_readModelNumber(&(vn.vs), modelNumber,
                                        sizeof(modelNumber))) != E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_readModelNumber failed.\n");
    processErrorReceived("Error reading model number.", error);
    return false;
  }
  printf("Model Number: %s\n", modelNumber);

  //（6）若没有将传感器切换到1 kHz模式 ，报错
  if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &oldHz)) !=E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
    processErrorReceived("Error reading async data output frequency.", error);
    return false;
  }

  //（7）若非零频率导致IMU以设置的频率输出ascii数据包以及二进制数据包 ，报错
  if ((error = VnSensor_writeAsyncDataOutputFrequency(&(vn.vs), 0, true)) !=E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_wrtieAsyncDataOutputFrequency failed.\n");
    processErrorReceived("Error writing async data output frequency.", error);
    return false;
  }
  if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &newHz)) !=E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
    processErrorReceived("Error reading async data output frequency.", error);
    return false;
  }
  printf("[rt_vectornav] Changed frequency from %d to %d Hz.\n", oldHz, newHz);

  //（8）若变相对航向模式以避免罗盘的怪异，报错
  if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
    processErrorReceived("Error reading VPE basic control.", error);
    return false;
  }
  strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
  printf("[rt_vectornav] Sensor was in mode: %s\n", strConversions);
  vpeReg.headingMode = VNHEADINGMODE_RELATIVE;
  if ((error = VnSensor_writeVpeBasicControl(&(vn.vs), vpeReg, true)) !=E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_writeVpeBasicControl failed.\n");
    processErrorReceived("Error writing VPE basic control.", error);
    return false;
  }
  if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE) 
  {
    processErrorReceived("Error reading VPE basic control.", error);
    printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
    return false;
  }
  strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
  printf("[rt_vectornav] Sensor now id mode: %s\n", strConversions);

  if ((error = VnSensor_readImuFilteringConfiguration(&(vn.vs), &filtReg)) !=E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_readGyroCompensation failed.\n");
  }
  printf("[rt_vectornav] AccelWindow: %d\n", filtReg.accelWindowSize);
  
  //（9）设置二进制输出消息类型 
  BinaryOutputRegister_initialize(
      &(vn.bor), ASYNCMODE_PORT2,
      4,  // divisor:  output frequency = 800/divisor
      (CommonGroup)(COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE |
                    COMMONGROUP_ACCEL),
      TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
      INSGROUP_NONE, GPSGROUP_NONE);

  if ((error = VnSensor_writeBinaryOutput1(&(vn.vs), &(vn.bor), true)) != E_NONE) 
  {
    printf("[rt_vectornav] VnSensor_writeBinaryOutput1 failed.\n");
    processErrorReceived("Error writing binary output 1.", error);
    return false;
  }

  // （10）安装处理程序
  VnSensor_registerAsyncPacketReceivedHandler(&(vn.vs), vectornav_handler,
                                              NULL);
  printf("[rt_vectornav] IMU is set up!\n");
  return true;
}

int got_first_vectornav_message = 0;




/*!
 * 功能：vectornav矢量导航获取新的数据包处理程序
 */
void vectornav_handler(void* userData, VnUartPacket* packet,
                       size_t running_index) 
{
  (void)userData;
  (void)running_index;
  vec4f quat;         //定义四元数
  vec3f omega;        //定义方向
  vec3f a;            //定义加速度

//（1）传感器报错提示
  if (VnUartPacket_type(packet) != PACKETTYPE_BINARY) 
  {
    printf("[vectornav_handler] got a packet that wasn't binary.\n");
    return;
  }

  if (!VnUartPacket_isCompatible(
          packet,
          (CommonGroup)(COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE |
                        COMMONGROUP_ACCEL),
          TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
          INSGROUP_NONE, GPSGROUP_NONE)) 
  {
    printf("[vectornav_handler] got a packet with the wrong type of data.\n");
    return;
  }

//（2）获取IMU的值：四元数(quat)、俯仰角(Omega)、加速度(a)
  quat  = VnUartPacket_extractVec4f(packet); //获取四元数(quat)
  omega = VnUartPacket_extractVec3f(packet); //获取俯仰角(Omega)
  a = VnUartPacket_extractVec3f(packet);     //获取加速度(a)

//（3）用IMU消息填充每条腿及关节的LCM通讯的数据
  for (int i = 0; i < 4; i++) //每条腿
  {
    vectornav_lcm_data.q[i] = quat.c[i];
    g_vn_data->quat[i] = quat.c[i];
  }

  for (int i = 0; i < 3; i++) //每个关节
  {
    vectornav_lcm_data.w[i] = omega.c[i];
    vectornav_lcm_data.a[i] = a.c[i];
    g_vn_data->gyro[i] = omega.c[i];
    g_vn_data->accelerometer[i] = a.c[i];
  }
//（4）发布IMU的LCM消息
  vectornav_lcm->publish("hw_vectornav", &vectornav_lcm_data);

//（5）调试IMU输出的数据，包括四元数、方向、加速度
#ifdef PRINT_VECTORNAV_DEBUG
  char strConversions[50];
  str_vec4f(strConversions, quat);
  printf("[QUAT] %s\n", strConversions);//四元数

  str_vec3f(strConversions, omega);
  printf("[OMEGA] %s\n", strConversions);//方向

  str_vec3f(strConversions, a);
  printf("[ACC] %s\n", strConversions);//加速度
#endif
}



/*!
 * 功能：vectornav矢量导航的错误回调函数
 * 备注：传感器消息报错的时候进来这里处理
 */
int processErrorReceived(const std::string& errorMessage, VnError errorCode) 
{
  //获取并打印错误信息
  char errorCodeStr[100];
  strFromVnError(errorCodeStr, errorCode);
  printf("%s\nVECTORNAV ERROR: %s\n", errorMessage.c_str(), errorCodeStr);
  return -1;
}
//#endif
