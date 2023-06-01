#include "process.h"
#include "concrete_states.h"

Process::Process(){
    processStatus = false;
    currentState = &Waiting::getInstance();
}

void Process::setState(State &newState){
    //currentState -> exit(this);
    currentState = &newState;
    //currentState -> enter(this);
}

void Process::execute(){
    currentState -> execute(this);
}

void Process::processOn(){
    processStatus = true;
}

void Process::processOff(){
    processStatus = false;
}

bool Process::getProcessStatus(){
    return processStatus;
}