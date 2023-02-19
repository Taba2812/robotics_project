#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"

int main(int argc, char **argv){
    JointConfiguration q;
    EndEffector ee;

    q << 1,2,3,4,5,6;
    ee.computeDirect(q);

    std::cout << ee.getPosition() << "\n\n";
    std::cout << ee.getOrientation() << "\n\n";

    BlockPosition bp;
    bp = ee.getPosition();

    Destination d(bp);
    d.computeInverse(ee);

    std::cout << d.getJointAngles() << "\n";

    return 0;
}