/*! @file FloatingBaseModel.cpp
 * 刚体浮基模型数据结构的实现
 *此类存储“刚体动力学”中描述的运动学树
 *Featherstone的“算法”（下载自
 *麻省理工学院互联网：https://www.springer.com/us/book/9780387743141）
 *
 * 树包括一个额外的“转子”身体为每个身体。这个转子是固定到父实体并具有传动约束
 *包括使用类似于第12章所述的技术
 *“机器人与多体动力学”作者：Jain。
 *实现：高度特定于每个刚体一个旋转转子的情况。转子具有与车身相同的接头类型，但具有额外的传动比应用于运动子空间的乘法器。与浮基什么都不做。
 */

#include "Dynamics/FloatingBaseModel.h"
#include "Math/orientation_tools.h"

#include <Utilities/Utilities_print.h>
#include <stdio.h>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ori;
using namespace spatial;
using namespace std;

/*!
 *在触点处施加单位测试力。返回inv触点惯性方向并计算出合成的量子点密度
 * @param gc_index             接触索引
 * @param force_ics_at_contact 用惯性坐标表示单位试验力
 * @params dstate -            结果加速度的输出参数
 * @return J H^{-1} J^T        1x1逆接触惯量
 */
template <typename T>
T FloatingBaseModel<T>::applyTestForce(const int gc_index,
                                       const Vec3<T> &force_ics_at_contact,
                                       DVec<T> &dstate_out) 
{
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();
  udpateQddEffects();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  dstate_out = DVec<T>::Zero(_nDof);

  // 旋转到绝对坐标
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D是扩展力传播矩阵的一列
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  T LambdaInv = 0;
  T tmp = 0;

  // 从头到脚
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
    dstate_out.tail(_nDof - 6) += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];

    //施加力传播器（见Pat的ICRA 2012论文）基本上，由于关节是灵活的，只有一部分力在前任身上有感觉。所以，当Xup^T像
    //关节被锁住了，ChiUp^T像关节一样把力发回是自由的
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }

  dstate_out.head(6) = _invIA5.solve(F);
  LambdaInv += F.dot(dstate_out.head(6));
  dstate_out.tail(_nDof - 6) += _qdd_from_base_accel * dstate_out.head(6);

  return LambdaInv;
}

/*!
 * 接触惯量算法的支持函数
 *计算“subqdd”组件产生的qdd,如果你熟悉费瑟斯通的稀疏操作点或者jain的创新分解：
 * H = L * D * L^T
 * 这些子量子点分量代表了中间的空间
 * i.e. if H^{-1} = L^{-T} * D^{-1} * L^{1}
 *那么我所说的subqdd=L^{-1}*tau这是个糟糕的解释。它需要乳胶。
 */
template <typename T>
void FloatingBaseModel<T>::udpateQddEffects() 
{
  if (_qddEffectsUpToDate) return;
  updateForcePropagators();
  _qdd_from_base_accel.setZero();
  _qdd_from_subqdd.setZero();

  // 传力通道
  //这个循环半等价于H上的cholesky分解，类似于费瑟斯通的稀疏操作空间算法，这些计算是为了把联合速率当作一个任务空间来处理
  //为此，F计算圆环对树下物体的动态影响

  for (size_t i = 6; i < _nDof; i++) {
    _qdd_from_subqdd(i - 6, i - 6) = 1;
    SVec<T> F = (_ChiUp[i].transpose() - _Xup[i].transpose()) * _S[i];
    size_t j = _parents[i];
    while (j > 5) {
      _qdd_from_subqdd(i - 6, j - 6) = _S[j].dot(F);
      F = _ChiUp[j].transpose() * F;
      j = _parents[j];
    }
    _qdd_from_base_accel.row(i - 6) = F.transpose();
  }
  _qddEffectsUpToDate = true;
}


/*!
*接触惯量算法的支持函数
*每个关节上的Comptues力传播器
 */
template <typename T>
void FloatingBaseModel<T>::updateForcePropagators() 
{
  if (_forcePropagatorsUpToDate) return;
  updateArticulatedBodies();
  for (size_t i = 6; i < _nDof; i++) {
    _ChiUp[i] = _Xup[i] - _S[i] * _Utot[i].transpose() / _d[i];
  }
  _forcePropagatorsUpToDate = true;
}


/*!
 * 动力学ABA算法的支持函数
 */
template <typename T>
void FloatingBaseModel<T>::updateArticulatedBodies() 
{
  if (_articulatedBodiesUpToDate) return;

  forwardKinematics();

  _IA[5] = _Ibody[5].getMatrix();

  // 循环1，沿着树
  for (size_t i = 6; i < _nDof; i++) {
    _IA[i] = _Ibody[i].getMatrix();  // initialize
    Mat6<T> XJrot = jointXform(_jointTypes[i], _jointAxes[i],
                               _state.q[i - 6] * _gearRatios[i]);
    _Xuprot[i] = XJrot * _Xrot[i];
    _Srot[i] = _S[i] * _gearRatios[i];
  }

  // 帕特最小约束的神奇原理（高斯也是！）
  for (size_t i = _nDof - 1; i >= 6; i--) {
    _U[i] = _IA[i] * _S[i];
    _Urot[i] = _Irot[i].getMatrix() * _Srot[i];
    _Utot[i] = _Xup[i].transpose() * _U[i] + _Xuprot[i].transpose() * _Urot[i];

    _d[i] = _Srot[i].transpose() * _Urot[i];
    _d[i] += _S[i].transpose() * _U[i];

    // 关节惯性递推
    Mat6<T> Ia = _Xup[i].transpose() * _IA[i] * _Xup[i] +
                 _Xuprot[i].transpose() * _Irot[i].getMatrix() * _Xuprot[i] -
                 _Utot[i] * _Utot[i].transpose() / _d[i];
    _IA[_parents[i]] += Ia;
  }

  _invIA5.compute(_IA[5]);
  _articulatedBodiesUpToDate = true;
}

// parents, gr, jtype, Xtree, I, Xrot, Irot,

/*!
 * 添加实体时填充成员变量
 * @param count (6 for fb, 1 for joint)
 */
template <typename T>
void FloatingBaseModel<T>::addDynamicsVars(int count) 
{
  if (count != 1 && count != 6) {
    throw std::runtime_error(
        "addDynamicsVars must be called with count=1 (joint) or count=6 "
        "(base).\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  SVec<T> zero6 = SVec<T>::Zero();
  Mat6<T> zero66 = Mat6<T>::Zero();

  SpatialInertia<T> zeroInertia(zero66);
  for (int i = 0; i < count; i++) {
    _v.push_back(zero6);
    _vrot.push_back(zero6);
    _a.push_back(zero6);
    _arot.push_back(zero6);
    _avp.push_back(zero6);
    _avprot.push_back(zero6);
    _c.push_back(zero6);
    _crot.push_back(zero6);
    _S.push_back(zero6);
    _Srot.push_back(zero6);
    _f.push_back(zero6);
    _frot.push_back(zero6);
    _fvp.push_back(zero6);
    _fvprot.push_back(zero6);
    _ag.push_back(zero6);
    _agrot.push_back(zero6);
    _IC.push_back(zeroInertia);
    _Xup.push_back(eye6);
    _Xuprot.push_back(eye6);
    _Xa.push_back(eye6);

    _ChiUp.push_back(eye6);
    _d.push_back(0.);
    _u.push_back(0.);
    _IA.push_back(eye6);

    _U.push_back(zero6);
    _Urot.push_back(zero6);
    _Utot.push_back(zero6);
    _pA.push_back(zero6);
    _pArot.push_back(zero6);
    _externalForces.push_back(zero6);
  }

  _J.push_back(D6Mat<T>::Zero(6, _nDof));
  _Jdqd.push_back(SVec<T>::Zero());

  resizeSystemMatricies();
}

/*!
 * Updates the size of H, C, Cqd, G, and Js when bodies are added
 * 添加实体时更新H、C、Cqd、G和Js的大小
 */
template <typename T>
void FloatingBaseModel<T>::resizeSystemMatricies() 
{
  _H.setZero(_nDof, _nDof);
  _C.setZero(_nDof, _nDof);
  _Cqd.setZero(_nDof);
  _G.setZero(_nDof);
  for (size_t i = 0; i < _J.size(); i++) {
    _J[i].setZero(6, _nDof);
    _Jdqd[i].setZero();
  }

  for (size_t i = 0; i < _Jc.size(); i++) {
    _Jc[i].setZero(3, _nDof);
    _Jcdqd[i].setZero();
  }
  _qdd_from_subqdd.resize(_nDof - 6, _nDof - 6);
  _qdd_from_base_accel.resize(_nDof - 6, 6);
  _state.q = DVec<T>::Zero(_nDof - 6);
  _state.qd = DVec<T>::Zero(_nDof - 6);
}



/*!
*创建浮动体
*@param惯性浮体的空间惯性
 */
template <typename T>
void FloatingBaseModel<T>::addBase(const SpatialInertia<T> &inertia) 
{
  if (_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  Mat6<T> zero6 = Mat6<T>::Zero();
  SpatialInertia<T> zeroInertia(zero6);
  // the floating base has 6 DOFs

  _nDof = 6;
  for (size_t i = 0; i < 6; i++) {
    _parents.push_back(0);
    _gearRatios.push_back(0);
    _jointTypes.push_back(JointType::Nothing);  // doesn't actually matter
    _jointAxes.push_back(CoordinateAxis::X);    // doesn't actually matter
    _Xtree.push_back(eye6);
    _Ibody.push_back(zeroInertia);
    _Xrot.push_back(eye6);
    _Irot.push_back(zeroInertia);
    _bodyNames.push_back("N/A");
  }

  _jointTypes[5] = JointType::FloatingBase;
  _Ibody[5] = inertia;
  _gearRatios[5] = 1;
  _bodyNames[5] = "Floating Base";

  addDynamicsVars(6);
}


/*!
 * 创建浮动体
 * @param mass 浮体质量
 * @param com  浮体质心
 * @param I    浮体转动惯量
 */
template <typename T>
void FloatingBaseModel<T>::addBase(T mass, const Vec3<T> &com,
                                   const Mat3<T> &I) 
{
  SpatialInertia<T> IS(mass, com, I);
  addBase(IS);
}


/*!
 * 向模型添加接地接触点
 * @param bodyID    向模型添加接地接触点
 * @param location  接触点的位置（在体坐标中）
 * @param isFoot    不管脚是不是真的。True if foot or not.
 * @return 接地触点的ID      The ID of the ground contact point
 */
template <typename T>
int FloatingBaseModel<T>::addGroundContactPoint(int bodyID,
                                                const Vec3<T> &location,
                                                bool isFoot) 
{
  if ((size_t)bodyID >= _nDof) {
    throw std::runtime_error(
        "addGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  // std::cout << "pt-add: " << location.transpose() << "\n";
  _gcParent.push_back(bodyID);
  _gcLocation.push_back(location);

  Vec3<T> zero3 = Vec3<T>::Zero();

  _pGC.push_back(zero3);
  _vGC.push_back(zero3);

  D3Mat<T> J(3, _nDof);
  J.setZero();

  _Jc.push_back(J);
  _Jcdqd.push_back(zero3);
  //_compute_contact_info.push_back(false);
  _compute_contact_info.push_back(true);

  // 将脚添加到脚列表
  if (isFoot) {
    _footIndicesGC.push_back(_nGroundContact);
    _compute_contact_info[_nGroundContact] = true;
  }

  resizeSystemMatricies();
  return _nGroundContact++;
}


/*!
 *将长方体的边界点添加到接触模型。假设盒子是
 *以身体坐标系的原点为中心，并与轴对齐。
 */
template <typename T>
void FloatingBaseModel<T>::addGroundContactBoxPoints(int bodyId,
                                                     const Vec3<T> &dims) 
{
  // addGroundContactPoint(bodyId, Vec3<T>( dims(0),  dims(1),  dims(2))/2);
  // addGroundContactPoint(bodyId, Vec3<T>(-dims(0),  dims(1),  dims(2))/2);
  // addGroundContactPoint(bodyId, Vec3<T>( dims(0), -dims(1),  dims(2))/2);
  // addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1),  dims(2))/2);

  addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), 0.) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), 0.) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), 0.) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), 0.) / 2);

  addGroundContactPoint(bodyId, Vec3<T>(dims(0), dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(dims(0), -dims(1), -dims(2)) / 2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1), -dims(2)) / 2);
}


/*!
 * 添加躯干
 * @param inertia       身体的惯性
 * @param rotorInertia  与主体相连的转子的惯性
 * @param gearRatio     机体和转子之间的传动比
 * @param parent        父体，也被假定为转子体连接到
 * @param jointType     关节类型（棱柱状或旋转）
 * @param jointAxis     父帧中的关节轴（X，Y，Z）
 * @param Xtree         从父对象到此实体的坐标转换
 * @param Xrot          从母体到转子的坐标变换
 * @return              主体的ID（可以用作父项）
 */
template <typename T>
int FloatingBaseModel<T>::addBody(const SpatialInertia<T> &inertia,
                                  const SpatialInertia<T> &rotorInertia,
                                  T gearRatio, int parent, JointType jointType,
                                  CoordinateAxis jointAxis,
                                  const Mat6<T> &Xtree, const Mat6<T> &Xrot) 
{
  if ((size_t)parent >= _nDof) {
    throw std::runtime_error(
        "addBody got invalid parent: " + std::to_string(parent) +
        " nDofs: " + std::to_string(_nDof) + "\n");
  }

  _parents.push_back(parent);
  _gearRatios.push_back(gearRatio);
  _jointTypes.push_back(jointType);
  _jointAxes.push_back(jointAxis);
  _Xtree.push_back(Xtree);
  _Xrot.push_back(Xrot);
  _Ibody.push_back(inertia);
  _Irot.push_back(rotorInertia);
  _nDof++;

  addDynamicsVars(1);

  return _nDof;
}


/*!
 * 添加躯干
 * @param inertia       身体的惯性
 * @param rotorInertia  与主体相连的转子的惯性
 * @param gearRatio     机体和转子之间的传动比
 * @param parent        父体，也被假定为转子体连接到
 * @param jointType     关节类型（棱柱状或旋转）
 * @param jointAxis     父帧中的关节轴（X，Y，Z）
 * @param Xtree         从父对象到此实体的坐标转换
 * @param Xrot          从母体到转子的坐标变换
 * @return              主体的ID（可以用作父项）
 */
template <typename T>
int FloatingBaseModel<T>::addBody(const MassProperties<T> &inertia,
                                  const MassProperties<T> &rotorInertia,
                                  T gearRatio, int parent, JointType jointType,
                                  CoordinateAxis jointAxis,
                                  const Mat6<T> &Xtree, const Mat6<T> &Xrot) 
{
  return addBody(SpatialInertia<T>(inertia), SpatialInertia<T>(rotorInertia),
                 gearRatio, parent, jointType, jointAxis, Xtree, Xrot);
}



template <typename T>
void FloatingBaseModel<T>::check() 
{
  if (_nDof != _parents.size())
    throw std::runtime_error("Invalid dof and parents length");
}

/*!
 * 计算非转子物体的总质量。
 */
template <typename T>
T FloatingBaseModel<T>::totalNonRotorMass() 
{
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Ibody[i].getMass();
  }
  return totalMass;
}



/*!
 *计算非转子物体的总质量。
 */
template <typename T>
T FloatingBaseModel<T>::totalRotorMass() 
{
  T totalMass = 0;
  for (size_t i = 0; i < _nDof; i++) {
    totalMass += _Irot[i].getMass();
  }
  return totalMass;
}



/*!
 *所有物体的正向运动学。计算Xup（从树上）和Xa
 *（从绝对值）还计算_S（运动子空间），_v（空间速度
 *链接坐标）和c（链接坐标中的科里奥利加速度）
 */
template <typename T>
void FloatingBaseModel<T>::forwardKinematics() 
{
  if (_kinematicsUpToDate) return;

  // 计算关节变换
  _Xup[5] = createSXform(quaternionToRotationMatrix(_state.bodyOrientation),
                         _state.bodyPosition);
  _v[5] = _state.bodyVelocity;
  for (size_t i = 6; i < _nDof; i++) {
    // 关节变形
    Mat6<T> XJ = jointXform(_jointTypes[i], _jointAxes[i], _state.q[i - 6]);
    _Xup[i] = XJ * _Xtree[i];
    _S[i] = jointMotionSubspace<T>(_jointTypes[i], _jointAxes[i]);
    SVec<T> vJ = _S[i] * _state.qd[i - 6];
    // 躯体i总速度
    _v[i] = _Xup[i] * _v[_parents[i]] + vJ;

    //转子相同
    Mat6<T> XJrot = jointXform(_jointTypes[i], _jointAxes[i],
                               _state.q[i - 6] * _gearRatios[i]);
    _Srot[i] = _S[i] * _gearRatios[i];
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _Xuprot[i] = XJrot * _Xrot[i];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;

    //科里奥利加速度
    _c[i] = motionCrossProduct(_v[i], vJ);
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
  }

  //从绝对变换计算
  for (size_t i = 5; i < _nDof; i++) {
    if (_parents[i] == 0) {
      _Xa[i] = _Xup[i];  // float base
    } else {
      _Xa[i] = _Xup[i] * _Xa[_parents[i]];
    }
  }

  //接地接触点
  // TODO : 最后我们将相同的Xa反转几次（就像身体上的8个点）。这不是超高效的。
  for (size_t j = 0; j < _nGroundContact; j++) {
    if (!_compute_contact_info[j]) continue;
    size_t i = _gcParent.at(j);
    Mat6<T> Xai = invertSXform(_Xa[i]);  // from link to absolute
    SVec<T> vSpatial = Xai * _v[i];

    // 在世界坐标系的脚的位置
    _pGC.at(j) = sXFormPoint(Xai, _gcLocation.at(j));
    _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
  }
  _kinematicsUpToDate = true;
}


/*!
*计算速度的接触雅可比矩阵（3xn矩阵）
*以绝对坐标表示的每个接触点
 */
template <typename T>
void FloatingBaseModel<T>::contactJacobians() 
{
  forwardKinematics();
  biasAccelerations();

  for (size_t k = 0; k < _nGroundContact; k++) {
    _Jc[k].setZero();
    _Jcdqd[k].setZero();

    //如果我们不在乎就跳过它
    if (!_compute_contact_info[k]) continue;

    size_t i = _gcParent.at(k);

    //旋转到绝对坐标
    Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
    Mat6<T> Xc = createSXform(Rai, _gcLocation.at(k));

    //偏压加速度
    SVec<T> ac = Xc * _avp[i];
    SVec<T> vc = Xc * _v[i];

    // 对古典的修正
    _Jcdqd[k] = spatialToLinearAcceleration(ac, vc);

    //在世界坐标上，线速度的行数
    D3Mat<T> Xout = Xc.template bottomRows<3>();

    //从头到脚
    while (i > 5) {
      _Jc[k].col(i) = Xout * _S[i];
      Xout = Xout * _Xup[i];
      i = _parents[i];
    }
    _Jc[k].template leftCols<6>() = Xout;
  }
}


/*!
*（支持函数）计算速度每个链路和转子积加速度
 */
template <typename T>
void FloatingBaseModel<T>::biasAccelerations() 
{
  if (_biasAccelerationsUpToDate) return;
  forwardKinematics();
  // 基底速度积加速度
  _avp[5] << 0, 0, 0, 0, 0, 0;

  //从底部到顶部
  for (size_t i = 6; i < _nDof; i++) {
    // 向外你逆运动扩展
    _avp[i] = _Xup[i] * _avp[_parents[i]] + _c[i];
    _avprot[i] = _Xuprot[i] * _avp[_parents[i]] + _crot[i];
  }
  _biasAccelerationsUpToDate = true;
}



/*!
 *功能：计算逆动力学中的广义引力（G）
 *@return G（_ndofx 1向量）
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::generalizedGravityForce() 
{
  compositeInertias();

  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];
  _ag[5] = _Xup[5] * aGravity;

//重力补偿力与加速所需的力相同反重力
  _G.template topRows<6>() = -_IC[5].getMatrix() * _ag[5];
  for (size_t i = 6; i < _nDof; i++) {
    _ag[i] = _Xup[i] * _ag[_parents[i]];
    _agrot[i] = _Xuprot[i] * _ag[_parents[i]];

    //本体和转子
    _G[i] = -_S[i].dot(_IC[i].getMatrix() * _ag[i]) -
            _Srot[i].dot(_Irot[i].getMatrix() * _agrot[i]);
  }
  return _G;
}


/*!
*逆动力学中广义科里奥利力的计算
*@return Cqd（_ndofx 1向量）
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::generalizedCoriolisForce() 
{
  biasAccelerations();

  //浮动基力
  Mat6<T> Ifb = _Ibody[5].getMatrix();
  SVec<T> hfb = Ifb * _v[5];
  _fvp[5] = Ifb * _avp[5] + forceCrossProduct(_v[5], hfb);

  for (size_t i = 6; i < _nDof; i++) {
    // 力作用于身体i
    Mat6<T> Ii = _Ibody[i].getMatrix();
    SVec<T> hi = Ii * _v[i];
    _fvp[i] = Ii * _avp[i] + forceCrossProduct(_v[i], hi);

    // 转子i受力
    Mat6<T> Ir = _Irot[i].getMatrix();
    SVec<T> hr = Ir * _vrot[i];
    _fvprot[i] = Ir * _avprot[i] + forceCrossProduct(_vrot[i], hr);
  }

  for (size_t i = _nDof - 1; i > 5; i--) {
    //沿关节的拔出力
    _Cqd[i] = _S[i].dot(_fvp[i]) + _Srot[i].dot(_fvprot[i]);

    //树下的扩展力
    _fvp[_parents[i]] += _Xup[i].transpose() * _fvp[i];
    _fvp[_parents[i]] += _Xuprot[i].transpose() * _fvprot[i];
  }

  // 浮基受力
  _Cqd.template topRows<6>() = _fvp[5];
  return _Cqd;
}



template <typename T>
Mat3<T> FloatingBaseModel<T>::getOrientation(int link_idx) 
{
  forwardKinematics();
  Mat3<T> Rai = _Xa[link_idx].template block<3, 3>(0, 0);
  Rai.transposeInPlace();
  return Rai;
}


template <typename T>
Vec3<T> FloatingBaseModel<T>::getPosition(const int link_idx)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // from link to absolute
  Vec3<T> link_pos = sXFormPoint(Xai, Vec3<T>::Zero());
  return link_pos;
}

template <typename T>
Vec3<T> FloatingBaseModel<T>::getPosition(const int link_idx, const Vec3<T> & local_pos)
{
  forwardKinematics();
  Mat6<T> Xai = invertSXform(_Xa[link_idx]); // from link to absolute
  Vec3<T> link_pos = sXFormPoint(Xai, local_pos);
  return link_pos;
}

template <typename T>
Vec3<T> FloatingBaseModel<T>::getLinearAcceleration(const int link_idx,
                                                    const Vec3<T> &point) 
{
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], point);
}

template <typename T>
Vec3<T> FloatingBaseModel<T>::getLinearAcceleration(const int link_idx) 
{
  forwardAccelerationKinematics();
  Mat3<T> R = getOrientation(link_idx);
  return R * spatialToLinearAcceleration(_a[link_idx], _v[link_idx], Vec3<T>::Zero());
}


template <typename T>
Vec3<T> FloatingBaseModel<T>::getLinearVelocity(const int link_idx,
                                                const Vec3<T> &point) 
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], point);
}


template <typename T>
Vec3<T> FloatingBaseModel<T>::getLinearVelocity(const int link_idx) 
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * spatialToLinearVelocity(_v[link_idx], Vec3<T>::Zero());
}



template <typename T>
Vec3<T> FloatingBaseModel<T>::getAngularVelocity(const int link_idx) 
{
  forwardKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  // Vec3<T> v3 =
  return Rai * _v[link_idx].template head<3>();
  ;
}


template <typename T>
Vec3<T> FloatingBaseModel<T>::getAngularAcceleration(const int link_idx) 
{
  forwardAccelerationKinematics();
  Mat3<T> Rai = getOrientation(link_idx);
  return Rai * _a[link_idx].template head<3>();
}

/*!
*（支承函数）计算复合刚体惯性
*每个子树中包含主体i和主体/转子
*机构一的所有继承人的惯量。
*（注：IC[i]不包含转子i）
 */
template <typename T>
void FloatingBaseModel<T>::compositeInertias() 
{
  if (_compositeInertiasUpToDate) return;

  forwardKinematics();
  //初始化
  for (size_t i = 5; i < _nDof; i++) {
    _IC[i].setMatrix(_Ibody[i].getMatrix());
  }

  //向后循环
  for (size_t i = _nDof - 1; i > 5; i--) {
    // 在树下传导惯性
    _IC[_parents[i]].addMatrix(_Xup[i].transpose() * _IC[i].getMatrix() *
                               _Xup[i]);
    _IC[_parents[i]].addMatrix(_Xuprot[i].transpose() * _Irot[i].getMatrix() *
                               _Xuprot[i]);
  }
  _compositeInertiasUpToDate = true;
}

/*!
*计算逆动力学公式中的质量矩阵（H）
*@return H（_ndofx _nDof矩阵）
 */
template <typename T>
DMat<T> FloatingBaseModel<T>::massMatrix() 
{
  compositeInertias();
  _H.setZero();

  //左上角是整个系统的锁定惯性
  _H.template topLeftCorner<6, 6>() = _IC[5].getMatrix();

  for (size_t j = 6; j < _nDof; j++) {
    //f=单位qdd_j所需的空间力
    SVec<T> f = _IC[j].getMatrix() * _S[j];
    SVec<T> frot = _Irot[j].getMatrix() * _Srot[j];
    _H(j, j) = _S[j].dot(f) + _Srot[j].dot(frot);

    //在树下扩展
    f = _Xup[j].transpose() * f + _Xuprot[j].transpose() * frot;
    size_t i = _parents[j];
    while (i > 5) {
      //在这里，f表示在{i}帧中
      _H(i, j) = _S[i].dot(f);
      _H(j, i) = _H(i, j);

      // 在树下扩展
      f = _Xup[i].transpose() * f;
      i = _parents[i];
    }

    //浮基受力
    _H.template block<6, 1>(0, j) = f;
    _H.template block<1, 6>(j, 0) = f.adjoint();
  }
  return _H;
}

template <typename T>
void FloatingBaseModel<T>::forwardAccelerationKinematics() 
{
  if (_accelerationsUpToDate) 
  {
    return;
  }

  forwardKinematics();
  biasAccelerations();

  //使用模型信息初始化重力
  SVec<T> aGravity = SVec<T>::Zero();
  aGravity.template tail<3>() = _gravity;

  //浮基空间力
  _a[5] = -_Xup[5] * aGravity + _dState.dBodyVelocity;

  //贯通接头
  for (size_t i = 6; i < _nDof; i++) {
  //空间加速度
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * _dState.qdd[i - 6] + _c[i];
    _arot[i] =
        _Xuprot[i] * _a[_parents[i]] + _Srot[i] * _dState.qdd[i - 6] + _crot[i];
  }
  _accelerationsUpToDate = true;
}

/*!
*计算系统的逆动力学
*@返回一个nDof x 1向量。前六项
*在底座上使用外螺纹扳手，其余的在底座上使用
*接头扭矩
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::inverseDynamics(
    const FBModelStateDerivative<T> &dState) 
{
  setDState(dState);
  forwardAccelerationKinematics();

  // 浮基空间力
  SVec<T> hb = _Ibody[5].getMatrix() * _v[5];
  _f[5] = _Ibody[5].getMatrix() * _a[5] + forceCrossProduct(_v[5], hb);

  // 通过关节循环
  for (size_t i = 6; i < _nDof; i++) {
    //空间动量
    SVec<T> hi = _Ibody[i].getMatrix() * _v[i];
    SVec<T> hr = _Irot[i].getMatrix() * _vrot[i];

    // 空间力
    _f[i] = _Ibody[i].getMatrix() * _a[i] + forceCrossProduct(_v[i], hi);
    _frot[i] =
        _Irot[i].getMatrix() * _arot[i] + forceCrossProduct(_vrot[i], hr);
  }

  DVec<T> genForce(_nDof);
  for (size_t i = _nDof - 1; i > 5; i--) {
    //沿关节拔出力的分量
    genForce[i] = _S[i].dot(_f[i]) + _Srot[i].dot(_frot[i]);

    //在树下扩展
    _f[_parents[i]] += _Xup[i].transpose() * _f[i];
    _f[_parents[i]] += _Xuprot[i].transpose() * _frot[i];
  }
  genForce.template head<6>() = _f[5];
  return genForce;
}

template <typename T>
void FloatingBaseModel<T>::runABA(const DVec<T> &tau,
                                  FBModelStateDerivative<T> &dstate) 
{
  (void)tau;
  forwardKinematics();
  updateArticulatedBodies();

  //为重力创建空间矢量
  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];

  //浮动底座铰接惯性
  SVec<T> ivProduct = _Ibody[5].getMatrix() * _v[5];
  _pA[5] = forceCrossProduct(_v[5], ivProduct);

  //循环1，沿着树
  for (size_t i = 6; i < _nDof; i++) {
    ivProduct = _Ibody[i].getMatrix() * _v[i];
    _pA[i] = forceCrossProduct(_v[i], ivProduct);

    //转子相同
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
    ivProduct = _Irot[i].getMatrix() * _vrot[i];
    _pArot[i] = forceCrossProduct(_vrot[i], ivProduct);
  }

  //根据外力调整pA
  for (size_t i = 5; i < _nDof; i++) {
    // TODO add if语句（如果力为零，则避免这些计算）
    Mat3<T> R = rotationFromSXform(_Xa[i]);
    Vec3<T> p = translationFromSXform(_Xa[i]);
    Mat6<T> iX = createSXform(R.transpose(), -R * p);
    _pA[i] = _pA[i] - iX.transpose() * _externalForces.at(i);
  }

  //帕特最小约束的神奇原理
  for (size_t i = _nDof - 1; i >= 6; i--) {
    _u[i] = tau[i - 6] - _S[i].transpose() * _pA[i] -
            _Srot[i].transpose() * _pArot[i] - _U[i].transpose() * _c[i] -
            _Urot[i].transpose() * _crot[i];

    //关节惯性递推
    SVec<T> pa =
        _Xup[i].transpose() * (_pA[i] + _IA[i] * _c[i]) +
        _Xuprot[i].transpose() * (_pArot[i] + _Irot[i].getMatrix() * _crot[i]) +
        _Utot[i] * _u[i] / _d[i];
    _pA[_parents[i]] += pa;
  }

  //包括重力和计算浮基加速度
  SVec<T> a0 = -aGravity;
  SVec<T> ub = -_pA[5];
  _a[5] = _Xup[5] * a0;
  SVec<T> afb = _invIA5.solve(ub - _IA[5].transpose() * _a[5]);
  _a[5] += afb;

  //关节加速度
  dstate.qdd = DVec<T>(_nDof - 6);
  for (size_t i = 6; i < _nDof; i++) {
    dstate.qdd[i - 6] =
        (_u[i] - _Utot[i].transpose() * _a[_parents[i]]) / _d[i];
    _a[i] = _Xup[i] * _a[_parents[i]] + _S[i] * dstate.qdd[i - 6] + _c[i];
  }

  //输出
  RotMat<T> Rup = rotationFromSXform(_Xup[5]);
  dstate.dBodyPosition =
      Rup.transpose() * _state.bodyVelocity.template block<3, 1>(3, 0);
  dstate.dBodyVelocity = afb;
  //上面的for循环中设置了qdd
}

/*!
 *在触点处施加单位测试力。返回inv触点惯性，这个方向并计算出合成的量子点密度
 * @param gc_index                  联系人索引
 * @param force_ics_at_contact      COE的单元测试
 * @params dstate -                 结果加速度的输出参数
 * @return  J H^{-1} J^T            1x1逆接触惯量
 */
template <typename T>
T FloatingBaseModel<T>::applyTestForce(const int gc_index,
                                       const Vec3<T> &force_ics_at_contact,
                                       FBModelStateDerivative<T> &dstate_out) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();
  udpateQddEffects();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  dstate_out.qdd.setZero();

  //旋转到绝对坐标
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  //D是扩展力传播矩阵的一列（见Wensing，2012 ICRA）
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // 从头到脚
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
    dstate_out.qdd += _qdd_from_subqdd.col(i - 6) * tmp / _d[i];

  //施加力传播器（见Pat的ICRA 2012论文）
  //基本上，由于关节是关节的，只有一部分力
  //在前任身上有感觉。所以，当Xup^T像
  //关节被锁住了，ChiUp^T像关节一样把力发回
  //是自由的
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }

  // TODO: 在更新计算体中仅执行一次QR
  dstate_out.dBodyVelocity = _invIA5.solve(F);
  LambdaInv += F.dot(dstate_out.dBodyVelocity);
  dstate_out.qdd += _qdd_from_base_accel * dstate_out.dBodyVelocity;

  return LambdaInv;
}

/*!
 * 计算接触惯性矩阵（mxm）的逆
 * @param force_ics_at_contact (3x1)
 *        e.g. if you want the cartesian inv. contact inertia in the z_ics
 *             force_ics_at_contact = [0 0 1]^T
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template <typename T>
T FloatingBaseModel<T>::invContactInertia(const int gc_index,
                                          const Vec3<T> &force_ics_at_contact) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  //旋转到绝对坐标
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  // D是扩展力传播矩阵的一列（见Wensing，2012 ICRA）
  SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

  double LambdaInv = 0;
  double tmp = 0;

  // 从头到脚
  while (i > 5) {
    tmp = F.dot(_S[i]);
    LambdaInv += tmp * tmp / _d[i];
  //施加力传播器（见Pat的ICRA 2012论文）
  //基本上，由于关节是关节的，只有一部分力
  //在前任身上有感觉。所以，当Xup^T像
  //关节被锁住了，ChiUp^T像关节一样把力发回
  //是自由的
    F = _ChiUp[i].transpose() * F;
    i = _parents[i];
  }
  LambdaInv += F.dot(_invIA5.solve(F));
  return LambdaInv;
}

/*!
 * 计算接触惯性矩阵（mxm）的逆
 * @param force_directions (6xm) each column denotes a direction of interest
 *        col = [ moment in i.c.s., force in i.c.s.]
 *        e.g. if you want the cartesian inv. contact inertia
 *             force_directions = [ 0_{3x3} I_{3x3}]^T
 *             if you only want the cartesian inv. contact inertia in one
 * direction then use the overloaded version.
 * @return the mxm inverse contact inertia J H^{-1} J^T
 */
template <typename T>
DMat<T> FloatingBaseModel<T>::invContactInertia(
    const int gc_index, const D6Mat<T> &force_directions) {
  forwardKinematics();
  updateArticulatedBodies();
  updateForcePropagators();

  size_t i_opsp = _gcParent.at(gc_index);
  size_t i = i_opsp;

  //旋转到绝对坐标
  Mat3<T> Rai = _Xa[i].template block<3, 3>(0, 0).transpose();
  Mat6<T> Xc = createSXform(Rai, _gcLocation.at(gc_index));

  D6Mat<T> D = Xc.transpose() * force_directions;

  size_t m = force_directions.cols();

  DMat<T> LambdaInv = DMat<T>::Zero(m, m);
  DVec<T> tmp = DVec<T>::Zero(m);

  // from tips to base
  while (i > 5) {
    tmp = D.transpose() * _S[i];
    LambdaInv += tmp * tmp.transpose() / _d[i];

    D = _ChiUp[i].transpose() * D;
    i = _parents[i];
  }

  // TODO: Only carry out the QR once within update Aritculated Bodies
  LambdaInv += D.transpose() * _invIA5.solve(D);

  return LambdaInv;
}

template class FloatingBaseModel<double>;
template class FloatingBaseModel<float>;
