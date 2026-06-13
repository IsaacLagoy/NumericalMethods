#ifndef SOLVER_MANAGER_H
#define SOLVER_MANAGER_H

#include "solver.h"
#include <vector>
#include <string>

struct NamedSolver {
    std::string name;
    Solver* solver;
};

class SolverManager {
private:
    std::vector<NamedSolver> solvers;
    int currentSolverIndex;

public:
    SolverManager(Robot* robot);
    ~SolverManager();

    void incSolver();
    void decSolver();
    void step();
};

#endif