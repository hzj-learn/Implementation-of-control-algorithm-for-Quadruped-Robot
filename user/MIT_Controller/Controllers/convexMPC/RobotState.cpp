#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

/**
 * 功能：把数组值转换成矩阵，方便计算连续状态空间方程
 * （1）p_   ：位置
 * （2）v_   ：速度
 * （3）q_   ：角速度
 * （4）w_   ：四元数
 * （5）r_   ：四脚从COM指向足端向量
 * （6）yaw_ ：偏航角
 */
void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];//位置
        this->v(i) = v_[i];//速度
        this->w(i) = w_[i];//角速度
    }

    this->q.w() = q_[0];//四元数
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;   //偏航角

    for(u8 rs = 0; rs < 3; rs++)    //四脚从COM指向足端向量
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix(); //机体旋转矩阵
    fpt yc = cos(yaw_);//Rz(fai)式（12）
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    Matrix<fpt,3,1> Id;             //机身坐标系下惯性矩阵
    Id << .07f, 0.26f, 0.242f;

    I_body.diagonal() = Id;
}


/**
 * 功能：机器人状态打印，包括位置、线速度、角速度、旋转角、偏航角、足端位置、身体惯量
 */
void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



