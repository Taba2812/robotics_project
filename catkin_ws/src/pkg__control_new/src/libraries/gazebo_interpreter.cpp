#include "gazebo_interpreter.h"

Gazebo::Interpreter::Interpreter() {
    this->n = 5;
}

void Gazebo::Interpreter::print(std::string str) {
    std::cout << str << " " << n << std::endl;
}