#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <vector>
#include <math.h>
#include <iostream>

namespace Bezier {

    typedef std::vector<float> Node;
    typedef std::vector<Node> NodeVector;

    class Curve {
        private:
            NodeVector nodes;
            int segment_quantity;
            int step;

            Node calculate(float t);
        public:
            Curve();
            Curve(Bezier::Node init, Bezier::Node dest, int seg);
            Curve(Node dest, int seg);
            Node getNext();
    };

    void printNode(Node node);
}

#endif
