#ifndef __MOTION_PLANNING_H__
#define __MOTION_PLANNING_H__

#include "inverse_kinematics.h"

#define VELOCITY 1

class Path{
private:
    JointConfiguration ic;
    JointConfiguration fc;
public:
    Path();
    Path(const JointConfiguration& jci, const JointConfiguration& jcf);
    JointConfiguration get_ic() const;
    JointConfiguration get_fc() const;
    void path_planning();
};

Path::Path(){
    ic << JointConfiguration::Zero();
    fc << JointConfiguration::Zero();
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

void Path::path_planning(){
    //linear phase: acceleration is zero, velocity is constant, position is linear
    //parabolic phase: acceleration is constant, velocity is linear and position is quadratic

    //we need the time between way-points: between q[i] and q[i+1] there is a time deltaT[i]
    //we need the duration of the blend phase at waypoint i, t(b)[i]

    //continuous trajectory: blend phases at waypoints, which replace part of the linear segments
    //around wayponints we have: a[i] = (v[i] - v[i-1]) / t(b)[i]
    //two blend phases cannot overlap with eath other: t(b)[i] + t(b)[i+1] <= 2*deltaT[i]

    //we need to make sure at the end of the blend we are at the same configuration that would have resulted from a linear trajectory
    //start the blend around i at time T[i] - t(b)[i]/2 at configuration q(T[i] - t(b)[i]/2)
    //apply constant acceleration a[i]
    //end the blend at time T[i] + t(b)[i]/2 and configuration q(T[i] + t(b)[i]/2)

    //between way-points i and i+1
    //the trajectory starts to follow the straight line at T[i] + t(b)[i]/2 and stop at T[i+1] - t(b)[i+1]/2
    //the duration of the linear phase between q[i] and q[i+1] is given by t(l)[i] = deltaT[i] - t(b)[i]/2 - t(b)[i+1]/2

    //the first and last blend phases need to be part of the trajectory completely
    //starts at t(b)[0]/2 before the first way-point
    //ends at t(b)[n]/2 after the last way-point

    //time of way-point i: T[i] = t(b)[1]/2 + sum[j=1,i-1](deltaT[j])
    //total duration of trajectory: t[f] = t(b)[1]/2 + sum[i=1, n-1](deltaT[i] + t(b)[n]/2)

    //RESULTING TRAJECTORY
    //blend phases: q(t) = q[i] + v[i-1]*(t-T[i]) + 1/2*a[i]*(t-T[i]+t(b)[i]/2) if T[i]-t(b)[i]/2 <= t <= T[i]+t(b)[i]/2 , i in 1...n
    //linear phases: q(t) = q[i] + v[i]*(t-T[i]) if T[i]+t(b)[i]/2 <= t <= T[i+1]-t(b)[i+1]/2 , i in 1...n-1
    //v[0] = v[n+1] = 0

    //first derivative of trajectory
    //blend phases: q.(t) = v[i-1] + a[i]*(t-T[i]-t(b)[i]/2) if T[i]-t(b)[i]/2 <= t <= T[i]+t(b)[i]/2 , i in 1...n
    //linear phases: q.(t) = v[i] if T[i]+t(b)[i]/2 <= t <= T[i+1]-t(b)[i+1]/2 , i in 1...n-1

    //second derivative of trajectory
    //blend phases: q..(t) = a[i] if T[i]-t(b)[i]/2 <= t <= T[i]+t(b)[i]/2 , i in 1...n
    //linear phases: q..(t) = 0 if T[i]+t(b)[i]/2 <= t <= T[i+1]-t(b)[i+1]/2 , i in 1...n-1

    /* This is the matlab file for p2p motion planning

    % Point 2 Point motion Plan
    % DK: function for direct kinematics
    % IK: function for inverse kineamtics
    % xEs, phiEs: starting configuration for the end effector
    % xEf, phiEf: final configuration for the end effector
    % T: duration of the motion paln
    % dt: sampling time
    function [Th,xE, phiE] = p2pMotionPlan(DK, IK, xEs, phiEs, xEf, phiEf, minT, maxT, dt)

    qEs = IK(xEs,eul2rotm(phiEs'));
    qEf = IK(xEf, eul2rotm(phiEf'));
    qEs = qEs(1,:);
    qEf = qEf(1,:);
    A = [];
    for i = 1:length(qEs(1,:)),
        M = [1, minT, minT^2, minT^3;
            0, 1, 2*minT, 3*minT^2;
            1, maxT, maxT^2, maxT^3;
            0, 1, 2*maxT, 3*maxT^2;];
        b = [qEs(i); 0; qEf(i); 0];
        a =  inv(M)*b;
        A = [A; a'];
    end
    Th = [];
    xE = [];
    phiE = [];
    for t = minT:dt:maxT,
        th = [t];
        for i = 1:length(qEs),
            q = A(i,1)+A(i,2)*t+A(i,3)*t*t+A(i,4)*t*t*t;
            th = [th q];
        end
        Th = [Th; th];
        [mx, mR] = DK(th(2:7));
        xE = [xE; t, mx'];
        phiE = [phiE; t, rotm2eul(mR)];
    end
    display('ciao');
    */

}

#endif
