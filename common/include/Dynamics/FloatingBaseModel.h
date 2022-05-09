/*! @file FloatingBaseModel.h
 *  @brief  刚体浮基模型数据结构的实现
 *
 * 此类存储“刚体动力学”中描述的运动学树
 *Featherstone的“算法”（下载自麻省理工学院互联网：https://www.springer.com/us/book/9780387743141）

 *树包括一个额外的“转子”身体为每个身体。这个转子是固定到父实体并具有传动约束。这是有效的
 *包括使用类似于第12章所述的技术“机器人与多体动力学”作者：Jain。注意这个实现是高度特定于每个刚体一个旋转转子的情况。
 * 转子具有与车身相同的接头类型，但具有额外的传动比应用于运动子空间的乘法器。与浮基什么都不做。
 */

#ifndef LIBBIOMIMETICS_FLOATINGBASEMODEL_H
#define LIBBIOMIMETICS_FLOATINGBASEMODEL_H

#include "Math/orientation_tools.h"
#include "SpatialInertia.h"
#include "spatial.h"

#include <eigen3/Eigen/StdVector>

#include <string>
#include <vector>

using std::vector;
using namespace ori;
using namespace spatial;

/*!
 * 浮动底座模型的状态（躯干和关节）
 */
template <typename T>
struct FBModelState 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quat<T> bodyOrientation;      //身体坐标系下，躯干方向
  Vec3<T> bodyPosition;         //身体坐标系下，躯干位置
  SVec<T> bodyVelocity;         //身体坐标系下，躯干速度
  DVec<T> q;                    //关节角度
  DVec<T> qd;                   //关节脚加速度

  void print() const 
  {
    printf("position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
           bodyPosition[2]);
  }
};


/*!
 *关节体算法在刚体浮体上的运行结果基本模型
 */
template <typename T>
struct FBModelStateDerivative 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> dBodyPosition;        //躯干位置
  SVec<T> dBodyVelocity;        //躯干速度
  DVec<T> qdd;                  //躯干加速度
};

/*!
 *类来表示带有转子和接地触点的浮动基础刚体模型。没有状态的概念。
 */
template <typename T>
class FloatingBaseModel 
{
 public:
  /*!
   * 使用默认重力初始化浮动基础模型函数
   */
  FloatingBaseModel() : _gravity(0, 0, -9.81) {}
  ~FloatingBaseModel() {}

  /*!
   * 添加浮动基函数。必须是添加的第一个主体，并且只能有一个
   */
  void addBase(const SpatialInertia<T>& inertia);

  /*!
   * 添加浮动基重载函数。必须是添加的第一个主体，并且只能有一个
   */
  void addBase(T mass, const Vec3<T>& com, const Mat3<T>& I);

  /*!
   * 添加碰撞点函数
   * @param bodyID   : 点所属的体（浮基体5）
   * @param location : 点在体坐标中的位置
   * @param isFoot   : 如果点是不是一英尺。只有脚才有
   * 机器人上的雅可比矩阵
   * @return 新点的碰撞点ID
   */
  int addGroundContactPoint(int bodyID, const Vec3<T>& location,
                            bool isFoot = false);

  /*!
   * 在实体周围添加边界框碰撞点函数
   * @param bodyId : body编号
   * @param dims   : 点的尺寸
   */
  void addGroundContactBoxPoints(int bodyId, const Vec3<T>& dims);

  /*!
   * 把躯干添加到树上函数
   * @param inertia        : 身体惯性（身体协调）
   * @param rotorInertia   : 转子惯性（转子坐标）
   * @param gearRatio      : 传动比。>1用于齿轮减速
   * @param parent         : 主体连接到的链接的主体ID
   * @param jointType      : 关节类型
   * @param jointAxis      : 关节轴线
   * @param Xtree          : 关节位置
   * @param Xrot           : 转子位置
   * @return               : bodyID
   */
  int addBody(const SpatialInertia<T>& inertia,
              const SpatialInertia<T>& rotorInertia, T gearRatio, int parent,
              JointType jointType, CoordinateAxis jointAxis,
              const Mat6<T>& Xtree, const Mat6<T>& Xrot);

  /*!
   * 把躯干添加到树上函数，重载
   * @param inertia        : 身体惯性（身体协调）
   * @param rotorInertia   : 转子惯性（转子坐标）
   * @param gearRatio      : 传动比。>1用于齿轮减速
   * @param parent         : 主体连接到的链接的主体ID
   * @param jointType      : 关节类型
   * @param jointAxis      : 关节轴线
   * @param Xtree          : 关节位置
   * @param Xrot           : 转子位置
   * @return               : bodyID
   */
  int addBody(const MassProperties<T>& inertia,
              const MassProperties<T>& rotorInertia, T gearRatio, int parent,
              JointType jointType, CoordinateAxis jointAxis,
              const Mat6<T>& Xtree, const Mat6<T>& Xrot);

  /*!
   *检查以确保尺寸函数
   */
  void check();

  /*!
   * 所有转子的总质量函数
   */
  T totalRotorMass();

  /*!
   * 非转子的所有物体的总质量函数
   */
  T totalNonRotorMass();

  const std::vector<int>& getParentVector() { return _parents; }

  const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>>&
  getBodyInertiaVector() 
  {
    return _Ibody;
  }

  const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>>&
  getRotorInertiaVector() 
  {
    return _Irot;
  }

  /*!
   * 设置重力函数
   */
  void setGravity(Vec3<T>& g) { _gravity = g; }

  void setContactComputeFlag(size_t gc_index, bool flag) 
  {
    _compute_contact_info[gc_index] = flag;
  }

  DMat<T> invContactInertia(const int gc_index,
                            const D6Mat<T>& force_directions);
  T invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,
                   FBModelStateDerivative<T>& dstate_out);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,
                   DVec<T>& dstate_out);

  void addDynamicsVars(int count);

  void resizeSystemMatricies();

  void setState(const FBModelState<T>& state) 
  {
    _state = state;

    _biasAccelerationsUpToDate = false;
    _compositeInertiasUpToDate = false;

    resetCalculationFlags();
  }
  void resetCalculationFlags() 
  {
    _articulatedBodiesUpToDate = false;
    _kinematicsUpToDate = false;
    _forcePropagatorsUpToDate = false;
    _qddEffectsUpToDate = false;
    _accelerationsUpToDate = false;
  }

  void setDState(const FBModelStateDerivative<T>& dState) 
  {
    _dState = dState;
    _accelerationsUpToDate = false;
  }

  Vec3<T> getPosition(const int link_idx, const Vec3<T> & local_pos);
  Vec3<T> getPosition(const int link_idx);


  Mat3<T> getOrientation(const int link_idx);
  Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T>& point);
  Vec3<T> getLinearVelocity(const int link_idx);

  Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T>& point);
  Vec3<T> getLinearAcceleration(const int link_idx);

  Vec3<T> getAngularVelocity(const int link_idx);
  Vec3<T> getAngularAcceleration(const int link_idx);

  void forwardKinematics();
  void biasAccelerations();
  void compositeInertias();
  void forwardAccelerationKinematics();
  void contactJacobians();

  DVec<T> generalizedGravityForce();
  DVec<T> generalizedCoriolisForce();
  DMat<T> massMatrix();
  DVec<T> inverseDynamics(const FBModelStateDerivative<T>& dState);
  void runABA(const DVec<T>& tau, FBModelStateDerivative<T>& dstate);

  size_t _nDof = 0;
  Vec3<T> _gravity;
  vector<int> _parents;
  vector<T> _gearRatios;
  vector<T> _d, _u;

  vector<JointType> _jointTypes;
  vector<CoordinateAxis> _jointAxes;
  vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
  vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody,
      _Irot;
  vector<std::string> _bodyNames;

  size_t _nGroundContact = 0;
  vector<size_t> _gcParent;
  vector<Vec3<T>> _gcLocation;
  vector<uint64_t> _footIndicesGC;

  vector<Vec3<T>> _pGC;
  vector<Vec3<T>> _vGC;

  vector<bool> _compute_contact_info;

  const DMat<T>& getMassMatrix() const { return _H; }
  const DVec<T>& getGravityForce() const { return _G; }
  const DVec<T>& getCoriolisForce() const { return _Cqd; }

  // void getPositionVelocity(
  // const int & link_idx, const Vec3<T> & local_pos,
  // Vec3<T> & link_pos, Vec3<T> & link_vel) const ;

  /// 开始算法支持变量
  FBModelState<T> _state;
  FBModelStateDerivative<T> _dState;

  vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
      _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot;

  vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
  vectorAligned<SVec<T>> _externalForces;

  vectorAligned<SpatialInertia<T>> _IC;
  vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp;

  DMat<T> _H, _C;
  DVec<T> _Cqd, _G;

  vectorAligned<D6Mat<T>> _J;
  vectorAligned<SVec<T>> _Jdqd;

  vectorAligned<D3Mat<T>> _Jc;
  vectorAligned<Vec3<T>> _Jcdqd;

  bool _kinematicsUpToDate = false;
  bool _biasAccelerationsUpToDate = false;
  bool _accelerationsUpToDate = false;

  bool _compositeInertiasUpToDate = false;

  void updateArticulatedBodies();
  void updateForcePropagators();
  void udpateQddEffects();

  void resetExternalForces() {
    for (size_t i = 0; i < _nDof; i++) {
      _externalForces[i] = SVec<T>::Zero();
    }
  }

  bool _articulatedBodiesUpToDate = false;
  bool _forcePropagatorsUpToDate = false;
  bool _qddEffectsUpToDate = false;

  DMat<T> _qdd_from_base_accel;
  DMat<T> _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;
};

#endif  // LIBBIOMIMETICS_FLOATINGBASEMODEL_H
