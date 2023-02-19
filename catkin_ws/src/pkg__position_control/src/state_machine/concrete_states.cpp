#include "concrete_states.h"

Waiting::~Waiting(){}

void Waiting::execute(Process *p){
    std::cout << "\n[WAITING] press a key to advance\n";
    std::cin.get();

    if(p -> getProcessStatus()){
        p -> processOff();
        p -> setState(Vision::getInstance());
    } else {
        usleep(WAITING_TIME);
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

    if(p -> getProcessStatus()){
        p -> processOff();
        p -> setState(Position::getInstance());
    } else {
        usleep(WAITING_TIME);
        p -> setState(Vision::getInstance());
    }
}

State &Vision::getInstance(){
    static Vision singleton;
    return singleton;
}

Position::~Position(){}

void Position::execute(Process *p){
    std::cout << "\n[POSITION]\n";
    
    if(p -> getProcessStatus()){
        p -> processOff();
        p -> setState(MotionTo::getInstance());
    } else {
        usleep(WAITING_TIME);
        p -> setState(Position::getInstance());
    }
}

State &Position::getInstance(){
    static Position singleton;
    return singleton;
}

MotionTo::~MotionTo(){}

void MotionTo::execute(Process *p){
    std::cout << "\n[MOVING TO BLOCK]\n";
    
    if(p -> getProcessStatus()){
        p -> processOff();
        p -> setState(Waiting::getInstance());
    } else {
        usleep(WAITING_TIME);
        p -> setState(MotionTo::getInstance());
    }
}

State &MotionTo::getInstance(){
    static MotionTo singleton;
    return singleton;
}