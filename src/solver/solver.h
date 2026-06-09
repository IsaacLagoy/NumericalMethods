#ifndef SOLVER_H
#define SOLVER_H

class Robot;

class Solver {
private:
    Robot* robot;

public:
    Solver(Robot* robot) : robot(robot) {}
    virtual ~Solver() = default;

    virtual void step() = 0;
};

#endif