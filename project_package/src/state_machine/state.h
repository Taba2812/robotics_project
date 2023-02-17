#ifndef __STATE_H__
#define __STATE_H__

#pragma once
#include "process.h"

class Process;

class State {
public:
    //virtual void enter(Process *p) = 0;
    virtual void execute(Process *p) = 0;
    //virtual void exit(Process *p) = 0;
    virtual ~State(){}
};

#endif
