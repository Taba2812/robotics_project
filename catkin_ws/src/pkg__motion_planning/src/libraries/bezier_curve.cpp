#include "bezier_curve.h"

void Bezier::printNode(Bezier::Node node) {
    std::cout <<  "x: " << node[0]
              << " y: " << node[1]
              << " z: " << node[2]
              << std::endl;
}

Bezier::Curve::Curve(Bezier::Node dest, int seg) {
    this->segment_quantity = seg;
    this->step = 0;

    //Takes the starting position as [0,0,0] and recive Delta Position based on the robot
    this->nodes.push_back({0,0,0});
    this->nodes.push_back({0,0,-3});
    this->nodes.push_back({dest[0], dest[1], dest[2] + 3});
    this->nodes.push_back(dest);
}

Bezier::Node Bezier::Curve::calculate(float t) {
    
    //k_x are the 4 coefficient for the Bernstein Polynomial form to calculate Bezier Curves
    float k_0 = (-1*pow(t,3)) + ( 3*pow(t,2)) - (3*t) + 1;
    float k_1 = ( 3*pow(t,3)) + (-6*pow(t,2)) + (3*t);
    float k_2 = (-3*pow(t,3)) + ( 3*pow(t,2));
    float k_3 =     pow(t,3);

    Bezier::Node result;

    for (int i = 0; i < 3; i++) {
        float value = nodes[0][i] * k_0 + nodes[1][i] * k_1 + nodes[2][i] * k_2 + nodes[3][i] * k_3;
        result.push_back(value);
    }

    return result;
}

Bezier::Node Bezier::Curve::getNext() { 
    Bezier::Node result = this->calculate((1/(float)this->segment_quantity) * this->step);
    this->step++;
    return result;
}