/*! @file ControlParameterInterface.h
 *  @brief 允许远程访问控制参数的类型，用于LCM/共享内存有响应/请求消息。
 * 机器人接收请求消息并用响应消息响应请求消息设置,参数或获取参数
 */

#ifndef PROJECT_CONTROLPARAMETERINTERFACE_H
#define PROJECT_CONTROLPARAMETERINTERFACE_H

#include <map>
#include "ControlParameters.h"

//控制参数请求种类
enum class ControlParameterRequestKind 
{
  GET_ROBOT_PARAM_BY_NAME,//通过参数名获取机器人
  SET_ROBOT_PARAM_BY_NAME,//通过参数名设置机器人
  GET_USER_PARAM_BY_NAME, //通过参数名获取用户
  SET_USER_PARAM_BY_NAME  //通过参数名设置用户
};

std::string controlParameterRequestKindToString(
    ControlParameterRequestKind request);

struct ControlParameterRequest 
{
  char name[CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH] =
      "";  // name of the parameter to set/get
  u64 requestNumber = UINT64_MAX;
  ControlParameterValue value;
  ControlParameterValueKind parameterKind;
  ControlParameterRequestKind requestKind;

  std::string toString() 
  {
    std::string result = "Request(" + std::to_string(requestNumber) + ") " +
                         controlParameterRequestKindToString(requestKind) +
                         " " +
                         controlParameterValueKindToString(parameterKind) +
                         " " + std::string(name) + " ";
    switch (requestKind) {
      case ControlParameterRequestKind::GET_USER_PARAM_BY_NAME:
        result += "user is: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
        result += "user to: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:
        result += "robot is: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
        result += "robot to: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      default:
        return result + " unknown request type!";
    }
  }
};

struct ControlParameterResponse 
{
  char name[CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH] = "";
  u64 requestNumber = UINT64_MAX;
  u64 nParameters = 0;
  ControlParameterValue value;
  ControlParameterValueKind parameterKind;
  ControlParameterRequestKind requestKind;

  bool isResponseTo(ControlParameterRequest& request) 
  {
    return requestNumber == request.requestNumber &&
           requestKind == request.requestKind &&
           std::string(name) == std::string(request.name);
  }

  std::string toString() 
  {
    std::string result = "Response(" + std::to_string(requestNumber) + ") " +
                         controlParameterRequestKindToString(requestKind) +
                         " " +
                         controlParameterValueKindToString(parameterKind) +
                         " " + std::string(name) + " ";

    switch (requestKind) {
      case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:
        result += "user to: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::GET_USER_PARAM_BY_NAME:
        result += "user is: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:
        result += "robot to: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:
        result += "robot is: ";
        result += controlParameterValueToString(value, parameterKind);
        return result;
      default:
        return result + " unknown request type!";
    }
  }
};

#endif  // PROJECT_CONTROLPARAMETERINTERFACE_H
