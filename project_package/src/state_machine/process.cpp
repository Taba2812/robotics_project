#include "process.h"
#include "concrete_states.h"

Process::Process(){
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