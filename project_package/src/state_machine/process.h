#ifndef __PROCESS_H__
#define __PROCESS_H__

#pragma once
#include "state.h"

class State;

class Process {
private:
    State *currentState;
    bool status;
public:
    Process();
    inline State *getCurrentState() const { return currentState; }
    void execute();
    void setState(State &newState);

    void setStatus(bool s);
    bool getStatus() const;
};

#endif
