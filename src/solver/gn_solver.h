#ifndef GAUSS_NEWTON_SOLVER_H
#define GAUSS_NEWTON_SOLVER_H

#include "solver.h"

class GaussNewtonSolver : public Solver {
public:
    GaussNewtonSolver(Robot* robot) : Solver(robot) {}
    void step() override;
};

#endif