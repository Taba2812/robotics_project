#ifndef __BEZIER_H__
#define __BEZIER_H__

#include "ur5.h"
#include <vector>
#include <eigen3/Eigen/Dense>
#include "direct_kinematics.h"
#include "inverse_kinematics.h"
#include "differential_kinematics.h"

class BezierCurve{
    private:
        int numPoints;
        std::vector<Eigen::Vector3d> curvePoints;
        Eigen::Vector3d P0,P1,P2;
        EndEffector ee;
        Destination dest;
    public:
        BezierCurve();
        BezierCurve(const Eigen::Vector3d& _P0, const Eigen::Vector3d& _P1, const Eigen::Vector3d& _P2, int _num_points);
        void generateCurve(Eigen::Vector3d& P0, Eigen::Vector3d& P1, Eigen::Vector3d& P2);
        void followCurve(EndEffector& ee, Destination& dest) const;
        int geNumPoints() const;
        Eigen::Vector3d getPoint(int index) const;
};

BezierCurve::BezierCurve(){
    numPoints = 0;
    curvePoints = {};
    P0, P1, P2 << Eigen::Vector3d::Zero();
}

BezierCurve::BezierCurve(const Eigen::Vector3d& _P0, const Eigen::Vector3d& _P1, const Eigen::Vector3d& _P2, int _numPoints){
    numPoints = _numPoints;
    curvePoints.reserve(numPoints);
    P0 = _P0; P1 = _P1; P2 = _P2;
}

int BezierCurve::geNumPoints() const{ 
    return this->numPoints;
}

Eigen::Vector3d BezierCurve::getPoint(int index) const {
    return curvePoints[index];
}

//genera la curva di bezier
void BezierCurve::generateCurve(Eigen::Vector3d& P0, Eigen::Vector3d& P1, Eigen::Vector3d& P2){
    //trovo i control points usando la quadratic bezier curve
    Eigen::Vector3d Q0 = P0 + (P1 - P0) * 2.0 / 3.0;
    Eigen::Vector3d Q1 = P2 + (P1 - P2) * 2.0 / 3.0;

    for (int i = 0; i < geNumPoints(); i++) {
      double t = (double)i / (double)(geNumPoints() - 1);     //valore tra 0 e 1, rappresenta uno step per un valore B della curva
      Eigen::Vector3d B = P0 * pow(1.0 - t, 2) + Q1 * 2.0 * t * (1.0 - t) + P2 * pow(t, 2);     //l'insieme delle B(t) è la nostra curva
      curvePoints.push_back(B);
    }
}

void BezierCurve::followCurve(EndEffector& ee, Destination& dest) const{
    for(int i = 0; i < geNumPoints(); i++){
        Eigen::Vector3d B = getPoint(i);
        ee.setPosition(B);
        //JointConfiguration q = dest.compute_inverse(ee);
        //inverse
        //direct(?)
        //differential
    }
}

#endif
