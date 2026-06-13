#include "solver_manager.h"

#include <iostream>

SolverManager::SolverManager(Robot* robot)
    : currentSolverIndex(0)
{
    solvers.push_back(NamedSolver{ "Gauss-Newton", new GaussNewtonSolver(robot) });
    solvers.push_back(NamedSolver{ "Levenberg-Marquardt", new LevenbergMarquardtSolver(robot, 1.0f) });
    solvers.push_back(NamedSolver{ "Gradient Descent", new GradientDescentSolver(robot, 0.01f) });
}

SolverManager::~SolverManager()
{
    solvers.clear();
}

void SolverManager::incSolver()
{
    currentSolverIndex = (currentSolverIndex + 1) % solvers.size();
    std::cout << "Using solver: " << solvers[currentSolverIndex].name << std::endl;
}

void SolverManager::decSolver()
{
    currentSolverIndex = (currentSolverIndex - 1 + solvers.size()) % solvers.size();
    std::cout << "Using solver: " << solvers[currentSolverIndex].name << std::endl;
}

void SolverManager::step()
{
    solvers[currentSolverIndex].solver->step();
}

void SolverManager::reset()
{
    solvers[currentSolverIndex].solver->reset();
}