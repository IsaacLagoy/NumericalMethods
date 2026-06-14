#include "solver.h"

GradientDescentSolver::GradientDescentSolver(Robot* robot, float stepSize) : 
    Solver(robot), 
    stepSize(stepSize) 
{}

void GradientDescentSolver::step()
{
    if (computeCost() < COST_EPSILON) return;

    const Eigen::VectorXf delta = -stepSize * computeGradient();
    robot->applyJointDeltas(unstackVec3s(delta));
}

void GradientDescentSolver::reset()
{
    // no-op
}