/*! @file ControlParameters.h
 *  @brief 模拟器和机器人增益/控制参数设置界面
 *  它们的设计是为了不经常更新。对于高频数据，考虑使用驱动程序输入或添加到Robot调试数据。
 *  ControlParameter：单个值，可以是double、float或s64。每个控制参数必须具有唯一的名称
 *  控制参数知道其类型以及是否已初始化控件参数必须初始化，从文件读取、从LCM读取或其他方式
 * 请参阅test_ControlParameters以了解其工作原理的示例
 */

#ifndef PROJECT_CONTROLPAREMETERS_H
#define PROJECT_CONTROLPAREMETERS_H

#include <map>
#include <mutex>
#include <string>
#include "Utilities/utilities.h"
#include "cTypes.h"

#define CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH 64

#define INIT_PARAMETER(name) param_##name(#name, name, collection)
#define DECLARE_PARAMETER(type, name) \
  type name;                          \
  ControlParameter param_##name;

enum class ControlParameterValueKind : u64 {
  FLOAT = 0,
  DOUBLE = 1,
  S64 = 2,
  VEC3_DOUBLE = 3,
  VEC3_FLOAT = 4
};

ControlParameterValueKind getControlParameterValueKindFromString(const std::string& str);

std::string controlParameterValueKindToString(
    ControlParameterValueKind valueKind);

union ControlParameterValuePtr {
  float* f;
  double* d;
  s64* i;
  float* vec3f;
  double* vec3d;
};

union ControlParameterValue {
  float f;
  double d;
  s64 i;
  float vec3f[3];
  double vec3d[3];
};

std::string controlParameterValueToString(ControlParameterValue v,
                                          ControlParameterValueKind kind);

class ControlParameter;

/*!
 * ControlParameterCollections包含所有控件参数的映射。
 */
class ControlParameterCollection {
 public:
  explicit ControlParameterCollection(const std::string& name) : _name(name) {}

  /*!
   *使用此选项在RobotControlParameters或SimulatorControlParameters
   * 这应该只是在初始化控制参数时使用
   */
  void addParameter(ControlParameter* param, const std::string& name) {
    if (mapContains(_map, name)) {
      printf(
          "[ERROR] ControlParameterCollection %s: tried to add parameter %s "
          "twice!\n",
          _name.c_str(), name.c_str());
      throw std::runtime_error("control parameter error");
    }
    _map[name] = param;
  }

  /*!
   *按名称查找控制参数。
   *不修改控制参数的设置字段！
   */
  ControlParameter& lookup(const std::string& name) {
    if (mapContains(_map, name)) {
      return *_map[name];
    } else {
      // for now:
      throw std::runtime_error("parameter " + name +
                               " wasn't found in parameter collection " +
                               _name);
    }
  }

  std::string
  printToIniString();               //以INI文件格式打印所有控制参数
  std::string printToYamlString();  //以YAML文件格式打印所有控制参数
  bool checkIfAllSet();             //所有的控制参数都初始化了吗？
  void clearAllSet();
  void deleteAll();

  std::map<std::string, ControlParameter*> _map;

 private:
  std::string _name;
};

class ControlParameter {
 public:
  /*!
   *控制参数的构造函数：
   *设置类型并添加到集合。
   *不“初始化”参数-这必须单独完成。
   */
  ControlParameter(const std::string& name, double& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.d = &value;
    _kind = ControlParameterValueKind::DOUBLE;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, float& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.f = &value;
    _kind = ControlParameterValueKind::FLOAT;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, s64& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.i = &value;
    _kind = ControlParameterValueKind::S64;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, Vec3<float>& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.vec3f = value.data();
    _kind = ControlParameterValueKind::VEC3_FLOAT;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, Vec3<double>& value,
                   ControlParameterCollection& collection,
                   const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.vec3d = value.data();
    _kind = ControlParameterValueKind::VEC3_DOUBLE;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, ControlParameterValueKind kind) {
    _name = name;
    truncateName();
    _kind = kind;
    _value.vec3d = (double*)&_staticValue;
  }

  void truncateName() {
    if (_name.length() > CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH) {
      printf("[Error] name %s is too long, shortening to ", _name.c_str());
      _name.resize(CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH - 1);
      printf("%s\n", _name.c_str());
    }
  }

  /*!
   *设置控制参数的初始值。
   *检查类型是否正确
   */
  void initializeDouble(double d) {
    if (_kind != ControlParameterValueKind::DOUBLE) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a double!");
    }
    _set = true;
    *_value.d = d;
  }



  void initializeFloat(float f) {
    if (_kind != ControlParameterValueKind::FLOAT) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a float!");
    }
    _set = true;
    *_value.f = f;
  }

  void initializeInteger(s64 i) {
    if (_kind != ControlParameterValueKind::S64) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as an integer!");
    }
    _set = true;
    *_value.i = i;
  }

  void initializeVec3f(const Vec3<float>& v) {
    if (_kind != ControlParameterValueKind::VEC3_FLOAT) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a vector3f");
    }
    _set = true;
    _value.vec3f[0] = v[0];
    _value.vec3f[1] = v[1];
    _value.vec3f[2] = v[2];
  }

  void initializeVec3d(const Vec3<double>& v) {
    if (_kind != ControlParameterValueKind::VEC3_DOUBLE) {
      throw std::runtime_error("Tried to initialize control parameter " +
                               _name + " as a vector3d");
    }
    _set = true;
    _value.vec3d[0] = v[0];
    _value.vec3d[1] = v[1];
    _value.vec3d[2] = v[2];
  }

  void set(ControlParameterValue value, ControlParameterValueKind kind) {
    if (kind != _kind) {
      throw std::runtime_error("type mismatch in set");
    }
    switch (kind) {
      case ControlParameterValueKind::FLOAT:
        *_value.f = value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
        *_value.d = value.d;
        break;
      case ControlParameterValueKind::S64:
        *_value.i = value.i;
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        _value.vec3f[0] = value.vec3f[0];
        _value.vec3f[1] = value.vec3f[1];
        _value.vec3f[2] = value.vec3f[2];
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        _value.vec3d[0] = value.vec3d[0];
        _value.vec3d[1] = value.vec3d[1];
        _value.vec3d[2] = value.vec3d[2];
        break;
      default:
        throw std::runtime_error("invalid kind");
    }
    _set = true;
  }

  ControlParameterValue get(ControlParameterValueKind kind) {
    ControlParameterValue value;
    if (kind != _kind) {
      throw std::runtime_error("type mismatch in get");
    }
    switch (_kind) {
      case ControlParameterValueKind::FLOAT:
        value.f = *_value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
        value.d = *_value.d;
        break;
      case ControlParameterValueKind::S64:
        value.i = *_value.i;
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        value.vec3f[0] = _value.vec3f[0];
        value.vec3f[1] = _value.vec3f[1];
        value.vec3f[2] = _value.vec3f[2];
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        value.vec3d[0] = _value.vec3d[0];
        value.vec3d[1] = _value.vec3d[1];
        value.vec3d[2] = _value.vec3d[2];
        break;
      default:
        throw std::runtime_error("invalid kind");
    }
    return value;
  }

  /*!
   * 将值转换为在YAML文件中工作的字符串
   */
  std::string toString() {
    std::string result;
    switch (_kind) {
      case ControlParameterValueKind::DOUBLE:
        result += numberToString(*_value.d);
        break;
      case ControlParameterValueKind::FLOAT:
        result += numberToString(*_value.f);
        break;
      case ControlParameterValueKind::S64:
        result += std::to_string(*_value.i);
        break;
      case ControlParameterValueKind::VEC3_FLOAT:
        result += "[";
        result += numberToString(_value.vec3f[0]) + ", ";
        result += numberToString(_value.vec3f[1]) + ", ";
        result += numberToString(_value.vec3f[2]) + "]";
        break;
      case ControlParameterValueKind::VEC3_DOUBLE:
        result += "[";
        result += numberToString(_value.vec3d[0]) + ", ";
        result += numberToString(_value.vec3d[1]) + ", ";
        result += numberToString(_value.vec3d[2]) + "]";
        break;
      default:
        result += "<unknown type " + std::to_string((u32)(_kind)) +
                  "> (add it yourself in ControlParameters.h!)";
        break;
    }
    return result;
  }

  bool setFromString(const std::string& value) {
    switch (_kind) {
      case ControlParameterValueKind::DOUBLE:
        *_value.d = std::stod(value);
        break;
      case ControlParameterValueKind::FLOAT:
        *_value.f = std::stof(value);
        break;
      case ControlParameterValueKind::S64:
        *_value.i = std::stoll(value);
        break;
      case ControlParameterValueKind::VEC3_FLOAT: {
        Vec3<float> v = stringToVec3<float>(value);
        _value.vec3f[0] = v[0];
        _value.vec3f[1] = v[1];
        _value.vec3f[2] = v[2];
      } break;
      case ControlParameterValueKind::VEC3_DOUBLE: {
        Vec3<double> v = stringToVec3<double>(value);
        _value.vec3d[0] = v[0];
        _value.vec3d[1] = v[1];
        _value.vec3d[2] = v[2];
      } break;
      default:
        return false;
    }
    return true;
  }

  bool _set = false;
  ControlParameterValuePtr _value;
  std::string _name;
  std::string _units;
  ControlParameterValueKind _kind;
  ControlParameterValue _staticValue;

 private:
};

/*!
 *参数组的父类
 *RobotParameters和SimulatorParameters从此类继承
 */
class ControlParameters {
 public:
  /*!
   *每个控制参数组必须有一个唯一的名称，这样ini文件就不会混在一起
   * @param name
   */
  ControlParameters(const std::string& name) : collection(name), _name(name) {}

  /*!
   * 如果为true，则所有参数都已以某种方式初始化
   */
  bool isFullyInitialized() { return collection.checkIfAllSet(); }

  /*!
   * 直接初始化给定的控制参数
   */
  void initializeDouble(const std::string& name, double d) {
    collection.lookup(name).initializeDouble(d);
  }

  void initializeFloat(const std::string& name, float f) {
    collection.lookup(name).initializeFloat(f);
  }

  void initializeInteger(const std::string& name, s64 i) {
    collection.lookup(name).initializeInteger(i);
  }

  void initializeVec3f(const std::string& name, Vec3<float>& v) {
    collection.lookup(name).initializeVec3f(v);
  }

  void initializeVec3d(const std::string& name, Vec3<double>& v) {
    collection.lookup(name).initializeVec3d(v);
  }

  void lockMutex() { _mutex.lock(); }

  void unlockMutex() { _mutex.unlock(); }

  void writeToIniFile(const std::string& path);
  void initializeFromIniFile(const std::string& path);
  void initializeFromYamlFile(const std::string& path);
  void defineAndInitializeFromYamlFile(const std::string& path);

  void writeToYamlFile(const std::string& path);

  std::string generateUnitializedList();

  ControlParameterCollection collection;

 protected:
  std::string _name;
  std::mutex _mutex;
};

#endif  // PROJECT_CONTROLPAREMETERS_H
