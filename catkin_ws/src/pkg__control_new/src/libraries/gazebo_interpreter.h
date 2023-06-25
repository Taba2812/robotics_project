#ifndef GAZEBO_INT
#define GAZEBO_INT

#include <string.h>
#include <iostream>
#include <Eigen/Dense>

namespace Gazebo {

    class Interpreter {
        private:
            int n;
        public:
            Interpreter();
            void print(std::string str);
            
    };

}

#endif