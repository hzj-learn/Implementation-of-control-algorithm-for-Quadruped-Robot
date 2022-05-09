#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"

//机器人状态方程

/*将数组值转换成矩阵，方便计算连续状态空间方程*/
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        fpt m = 9;
        //fpt m = 50.236; //DH
    //private:
};
#endif
