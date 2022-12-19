#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

#include "direct_kinematics.h"

void compute_inverse(const EndEffector& ee){
    //populate T06
    Eigen::VectorXd cmp;
    cmp.resize(4,1);
    cmp << 0,0,0,1;

    Eigen::MatrixXd T06;
    T06.resize(3,3);
    T06 << ee.orientation;

    T06.conservativeResize(Eigen::NoChange, T06.cols()+1);
    T06.col(T06.cols()-1) = ee.position;

    T06.conservativeResize(T06.rows()+1, Eigen::NoChange);
    T06.row(T06.rows()-1) = cmp;
}

#endif
