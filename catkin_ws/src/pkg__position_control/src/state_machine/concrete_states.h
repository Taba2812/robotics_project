#ifndef __CONCRETE_STATES_H__
#define __CONCRETE_STATES_H__

#pragma once
#include "state.h"
#include "process.h"
#include "../headers/ur5.h"
#include <iostream>
#include <unistd.h>

class Waiting : public State {
private:
    Waiting(){ stateCode = 0; };
    Waiting(const Waiting &w);
    virtual ~Waiting();
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Vision : public State {
private:
    Vision(){ stateCode = 1; }
    Vision(const Vision &v);
    virtual ~Vision();
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Position : public State {
private:
    Position(){ stateCode = 2; }
    Position(const Position &p);
    virtual ~Position();
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Motion : public State {
private:
    Motion(){ stateCode = 3; }
    Motion(const Motion &m);
    virtual ~Motion();
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

#endif
