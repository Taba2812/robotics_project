#ifndef __CONCRETE_STATES_H__
#define __CONCRETE_STATES_H__

#pragma once
#include "state.h"
#include "process.h"
#include <iostream>

class Waiting : public State {
private:
    Waiting(){}
    Waiting(const Waiting &w);
    virtual ~Waiting();
    Waiting &operator=(const Waiting &w);
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Vision : public State {
private:
    Vision(){}
    Vision(const Vision &v);
    virtual ~Vision();
    Vision &operator=(const Vision &v);
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Position : public State {
private:
    Position(){}
    Position(const Position &p);
    virtual ~Position();
    Position &operator=(const Position &p);
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

class Motion : public State {
private:
    Motion(){}
    Motion(const Motion &m);
    virtual ~Motion();
    Motion &operator=(const Motion &m);
public:
    //void enter(Process *p);
    void execute(Process *p);
    //void exit(Process *p);
    static State &getInstance();
};

#endif
