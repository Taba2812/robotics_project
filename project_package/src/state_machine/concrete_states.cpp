#include "concrete_states.h"

Waiting::~Waiting(){}

void Waiting::execute(Process *p){
    std::cout << "\n[WAITING]\n";
    if(p -> getStatus()){
        p -> setState(Vision::getInstance());
    } else {
        p -> setStatus(true);
        p -> setState(Waiting::getInstance());
    }
}

State &Waiting::getInstance(){
    static Waiting singleton;
    return singleton;
}

Vision::~Vision(){}

void Vision::execute(Process *p){
    std::cout << "\n[VISION]\n";
    p -> setState(Position::getInstance());
}

State &Vision::getInstance(){
    static Vision singleton;
    return singleton;
}

Position::~Position(){}

void Position::execute(Process *p){
    std::cout << "\n[POSITION]\n";
    p -> setState(Motion::getInstance());
}

State &Position::getInstance(){
    static Position singleton;
    return singleton;
}

Motion::~Motion(){}

void Motion::execute(Process *p){
    std::cout << "\n[MOTION]\n";
    p -> setState(Waiting::getInstance());
}

State &Motion::getInstance(){
    static Motion singleton;
    return singleton;
}