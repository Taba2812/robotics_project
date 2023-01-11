#include "headers/direct_kinematics.h"
#include "headers/inverse_kinematics.h"

int main(int argc, char **argv){
    JointConfiguration q;
    EndEffector ee;

    q << 1,2,3,4,5,6;
    ee.compute_direct(q);

    std::cout << ee.get_position() << "\n\n";
    std::cout << ee.get_orientation() << "\n\n";

    BlockPosition bp;
    bp = ee.get_position();

    Destination d(bp);
    d.compute_inverse(ee);

    std::cout << d.get_joint_angles() << "\n";

    return 0;
}