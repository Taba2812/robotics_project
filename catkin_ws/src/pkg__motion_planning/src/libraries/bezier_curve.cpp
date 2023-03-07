#include "bezier_curve.h"

Bezier::Node Bezier::Curve::calculate(float t) {
    //k_x are the 4 coefficient for the Bernstein Polynomial form to calculate Bezier Curves

    float k_0 = (-1*pow(t,3)) + (3*pow(t, 2)) - ((3*t)+1);
    float k_1 = (3*pow(t,3))  + (-6*pow(t,2)) - (3 * t);
    float k_2 = (-3*pow(t,3)) + (3*pow(t, 2));
    float k_3 = pow(t,3);
}