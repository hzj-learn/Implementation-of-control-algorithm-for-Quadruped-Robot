/*!
 *  四足机器人包含参数的数据结构
 */

#include "Dynamics/Quadruped.h"
#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"

using namespace ori;
using namespace spatial;

/*!
 * 建立四足动物的浮基模型函数
 * 我们假设猎豹的身体（不包括转子）可以建模为均匀分布的盒子
 */
template <typename T>
bool Quadruped<T>::buildModel(FloatingBaseModel<T>& model) 
{
  /*(1)定义四足机器人的长宽高*/
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);

  /*(2)添加浮动机体的惯性参数*/
  model.addBase(_bodyInertia);

  /*(3)添加浮动机体的接触点*/
  model.addGroundContactBoxPoints(5, bodyDims);

  const int baseID = 5;
  int bodyID = baseID;
  T sideSign = -1;

  Mat3<T> I3 = Mat3<T>::Identity();

  /*(4)遍历四条腿，并设置每条腿的三个关节参数*/ 
  for (int legID = 0; legID < 4; legID++)
  {
  ////////////////////////////////////*1）外展关节*/////////////////////////////////////////////////////////////////
    bodyID++;  
    Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID));//计算从旋转和平移创建空间坐标变换
    Mat6<T> xtreeAbadRotor =
        createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));
    //添加躯干
    if (sideSign < 0) 
    {
      model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                    _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _abadGearRatio, baseID, JointType::Revolute,
                    CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
    } 
    else 
    {
      model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio, baseID,
                    JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                    xtreeAbadRotor);
    }


////////////////////////////////////*2）髋关节*/////////////////////////////////////////////////////////////////
    bodyID++;  
    Mat6<T> xtreeHip =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),//计算从旋转和平移创建空间坐标变换
                     withLegSigns<T>(_hipLocation, legID));
    Mat6<T> xtreeHipRotor =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withLegSigns<T>(_hipRotorLocation, legID));
    //添加躯干
    if (sideSign < 0) 
    {
      model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),
                    _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _hipGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
    } 
    else 
    {
      model.addBody(_hipInertia, _hipRotorInertia, _hipGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                    xtreeHipRotor);
    }
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_hipLinkLength));//添加膝盖-地面接触点



    ////////////////////////////////////*3）膝关节*/////////////////////////////////////////////////////////////////
    bodyID++;   
    Mat6<T> xtreeKnee = createSXform(I3, _kneeLocation);//计算从旋转和平移创建空间坐标变换
    Mat6<T> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
    //添加躯干
    if (sideSign < 0) 
    {
      model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);
    } 
    else 
    {
      model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                    xtreeKneeRotor);
    }
   ////////////////////////////////////*4）添加脚*/////////////////////////////////////////////////////////////////
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

    sideSign *= -1;
  }
  /*(5)添加机器人重量（力）参数*/ 
  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return true;
}

/*!
 * 建立四足动物的浮基模型函数，重载函数
 */
template <typename T>
FloatingBaseModel<T> Quadruped<T>::buildModel() 
{
  FloatingBaseModel<T> model;

  // we assume the cheetah's body (not including rotors) can be modeled as a
  // uniformly distributed box.
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
  // model.addBase(_bodyMass, Vec3<T>(0,0,0), rotInertiaOfBox(_bodyMass,
  // bodyDims));
  model.addBase(_bodyInertia);
  // add contact for the cheetah's body
  model.addGroundContactBoxPoints(5, bodyDims);

  const int baseID = 5;
  int bodyID = baseID;
  T sideSign = -1;

  Mat3<T> I3 = Mat3<T>::Identity();

  // loop over 4 legs
  for (int legID = 0; legID < 4; legID++) {
    // Ab/Ad joint
    //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
    //  rotorInertia, T gearRatio,
    //              int parent, JointType jointType, CoordinateAxis jointAxis,
    //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
    bodyID++;
    Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID));
    Mat6<T> xtreeAbadRotor =
        createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));

    if (sideSign < 0) {
      model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                    _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _abadGearRatio, baseID, JointType::Revolute,
                    CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
    } else {
      model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio, baseID,
                    JointType::Revolute, CoordinateAxis::X, xtreeAbad,
                    xtreeAbadRotor);
    }

    // Hip Joint
    bodyID++;
    Mat6<T> xtreeHip =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withLegSigns<T>(_hipLocation, legID));
    Mat6<T> xtreeHipRotor =
        createSXform(coordinateRotation<T>(CoordinateAxis::Z, T(M_PI)),
                     withLegSigns<T>(_hipRotorLocation, legID));
    if (sideSign < 0) {
      model.addBody(_hipInertia.flipAlongAxis(CoordinateAxis::Y),
                    _hipRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _hipGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeHip, xtreeHipRotor);
    } else {
      model.addBody(_hipInertia, _hipRotorInertia, _hipGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeHip,
                    xtreeHipRotor);
    }

    // add knee ground contact point
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_hipLinkLength));

    // Knee Joint
    bodyID++;
    Mat6<T> xtreeKnee = createSXform(I3, _kneeLocation);
    Mat6<T> xtreeKneeRotor = createSXform(I3, _kneeRotorLocation);
    if (sideSign < 0) {
      model.addBody(_kneeInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                    _kneeGearRatio, bodyID - 1, JointType::Revolute,
                    CoordinateAxis::Y, xtreeKnee, xtreeKneeRotor);
    } else {
      model.addBody(_kneeInertia, _kneeRotorInertia, _kneeGearRatio, bodyID - 1,
                    JointType::Revolute, CoordinateAxis::Y, xtreeKnee,
                    xtreeKneeRotor);
    }

    // add foot
    model.addGroundContactPoint(bodyID, Vec3<T>(0, 0, -_kneeLinkLength), true);

    sideSign *= -1;
  }

  Vec3<T> g(0, 0, -9.81);
  model.setGravity(g);

  return model;
}

/*!
 * 根据向量V所属的支路翻转向量V元素的符号函数
 */
template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID) 
{
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID) {
    case 0:
      return Vec3<T>(v[0], -v[1], v[2]);
    case 1:
      return Vec3<T>(v[0], v[1], v[2]);
    case 2:
      return Vec3<T>(-v[0], -v[1], v[2]);
    case 3:
      return Vec3<T>(-v[0], v[1], v[2]);
    default:
      throw std::runtime_error("Invalid leg id!");
  }
}

/*!
 * 建立电机执行器模型函数
 */
template <typename T>
std::vector<ActuatorModel<T>> Quadruped<T>::buildActuatorModels() 
{
  std::vector<ActuatorModel<T>> models;
  models.emplace_back(_abadGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_hipGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  models.emplace_back(_kneeGearRatio, _motorKT, _motorR, _batteryV,
                      _jointDamping, _jointDryFriction, _motorTauMax);
  return models;
}

template class Quadruped<double>;
template class Quadruped<float>;
