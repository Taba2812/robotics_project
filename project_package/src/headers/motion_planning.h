#ifndef __MOTION_PLANNING_H__
#define __MOTION_PLANNING_H__

#include "inverse_kinematics.h"

class Path{
private:
    JointConfiguration ic;
    JointConfiguration fc;
public:
    Path();
    Path(const JointConfiguration& jci, const JointConfiguration& jcf);
    JointConfiguration get_ic() const;
    JointConfiguration get_fc() const;
};

Path::Path(){
    ic << Eigen::MatrixXd::Zero();
    fc << Eigen::MatrixXd::Zero();
}

Path::Path(const JointConfiguration& jci, const JointConfiguration& jcf){
    ic = jci;
    fc = jcf;
}

JointConfiguration Path::get_ic() const{
    return this -> ic;
}

JointConfiguration Path::get_fc() const{
    return this -> fc;
}

#endif
