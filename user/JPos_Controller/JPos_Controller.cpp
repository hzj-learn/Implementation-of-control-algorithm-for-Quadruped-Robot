//摆动腿轨迹跟踪控制器C++文件
#include "JPos_Controller.hpp"


/*!
 * 功能：设定KP\KD反馈矩阵参数，并发布每个关节发布位置、速度、力矩、及对应KP\KD五个参数
 */
void JPos_Controller::runController()
{
  ///////////////////////////*（1）定义并给定关节电机的KP\KD反馈增益矩阵*//////////////////////////////////////
  Mat3<float> kpMat;
  Mat3<float> kdMat;
  //////////////////////////*（1.1）通过给定的方法赋值KP\KD反馈增益矩阵*//////////////////////////
  //kpMat << 20, 0, 0, 0, 20, 0, 0, 0, 20;
  //kdMat << 2.1, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  //////////////////////////*(1.2)通过用户设定的方法赋值KP\KD反馈增益矩阵（可改，灵活度高一点）*////
  kpMat << userParameters.kp, 0, 0, 0,  userParameters.kp, 0, 0, 0,  userParameters.kp;
  kdMat <<  userParameters.kd, 0, 0, 0, userParameters.kd, 0, 0, 0, userParameters.kd;

  //////////////////////////*（2）从初始化数据中搬运12个电机参数*//////////////////////////////
  static int iter(0);//定义迭代次数
  ++iter;
  if(iter < 10)//运行10次
  {
    for(int leg(0); leg<4; ++leg)     //四条腿
    {
      for(int jidx(0); jidx<3; ++jidx)//每条腿的三个关节
      {
        _jpos_ini[3*leg+jidx] = _legController->datas[leg].q[jidx];   //这种方式是指令第几条腿的第几个个关节，12个关节依次搬运电机参数
      }
    }
  }

  //////////////////////////*（3）在腿部控制器中设定电机关节最大转矩*/////////////////////////////
  _legController->_maxTorque = 150;       

  //////////////////////////*（4）使能腿部控制器*///////////////////////////////////////////////
  _legController->_legsEnabled = true;    

  /////////////////////////*（5）校正编码器*///////////
  if(userParameters.calibrate > 0.4) //用户的校正参数>0.4时
  {
    _legController->_calibrateEncoders = userParameters.calibrate;//使用用户的参数校正值来校正编码器
  } 

  else                               //用户的参数校正<0.4时
  {
    if(userParameters.zero > 0.5) //用户的零位参数>0.5时
    {
      _legController->_zeroEncoders = true;//编码器置零
    } 

    else                          //用户的零位参数<0.5时
    {
      _legController->_zeroEncoders = false;//编码器不置零

    ///////////////////////////*(6)发布每个关节发布位置、速度、力矩、及对应KP\KD五个参数*//////////////
      for(int leg(0); leg<4; ++leg)           
      {
        for(int jidx(0); jidx<3; ++jidx)
        {
          float pos = std::sin(.001f * iter);
          _legController->commands[leg].qDes[jidx] = pos;                               //关节的位置指令
          _legController->commands[leg].qdDes[jidx] = 0.;                               //关节的速度指令
          _legController->commands[leg].tauFeedForward[jidx] = userParameters.tau_ff;   //关节的前馈力矩指令
        }
        _legController->commands[leg].kpJoint = kpMat;                                  //关节的KP反馈增益矩阵
        _legController->commands[leg].kdJoint = kdMat;                                  //关节的KD反馈增益矩阵
      }
    }
  }
}
