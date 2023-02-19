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
        int num_points;
        std::vector<Eigen::Vector3d> curve_points;
        Eigen::Vector3d P0,P1,P2;
        EndEffector ee;
        Destination dest;
    public:
        BezierCurve();
        BezierCurve(const Eigen::Vector3d& _P0, const Eigen::Vector3d& _P1, const Eigen::Vector3d& _P2, int _num_points);
        void generate_curve(Eigen::Vector3d& P0, Eigen::Vector3d& P1, Eigen::Vector3d& P2);
        void follow_curve(EndEffector& ee, Destination& dest) const;
        int get_numpoints() const;
        Eigen::Vector3d get_point(int index) const;
};

BezierCurve::BezierCurve(){
    num_points = 0;
    curve_points = {};
    P0, P1, P2 << Eigen::Vector3d::Zero();
}

BezierCurve::BezierCurve(const Eigen::Vector3d& _P0, const Eigen::Vector3d& _P1, const Eigen::Vector3d& _P2, int _num_points){
    num_points = _num_points;
    curve_points.reserve(num_points);
    P0 = _P0; P1 = _P1; P2 = _P2;
}

int BezierCurve::get_numpoints() const{ 
    return this->num_points;
}

Eigen::Vector3d BezierCurve::get_point(int index) const {
    return curve_points[index];
}

//genera la curva di bezier
void BezierCurve::generate_curve(Eigen::Vector3d& P0, Eigen::Vector3d& P1, Eigen::Vector3d& P2){
    //trovo i control points usando la quadratic bezier curve
    Eigen::Vector3d Q0 = P0 + (P1 - P0) * 2.0 / 3.0;
    Eigen::Vector3d Q1 = P2 + (P1 - P2) * 2.0 / 3.0;

    for (int i = 0; i < get_numpoints(); i++) {
      double t = (double)i / (double)(get_numpoints() - 1);     //valore tra 0 e 1, rappresenta uno step per un valore B della curva
      Eigen::Vector3d B = P0 * pow(1.0 - t, 2) + Q1 * 2.0 * t * (1.0 - t) + P2 * pow(t, 2);     //l'insieme delle B(t) è la nostra curva
      curve_points.push_back(B);
    }
}

void BezierCurve::follow_curve(EndEffector& ee, Destination& dest) const{
    for(int i = 0; i < get_numpoints(); i++){
        Eigen::Vector3d B = get_point(i);
        ee.set_position(B);
        //JointConfiguration q = dest.compute_inverse(ee);
        //inverse
        //direct(?)
        //differential
    }
}

#endif
