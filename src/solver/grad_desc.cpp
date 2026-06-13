#include "solver.h"

GradientDescentSolver::GradientDescentSolver(Robot* robot, float stepSize) : 
    Solver(robot), 
    stepSize(stepSize) 
{}

void GradientDescentSolver::step()
{
    const Eigen::VectorXf residual = computeResidual();
    if (residual.norm() < COST_EPSILON) return;

    const Eigen::MatrixXf jacobian = currJacobian();
    const Eigen::VectorXf gradient = jacobian.transpose() * residual;
    const Eigen::VectorXf delta = -stepSize * gradient;
    robot->applyJointDeltas(unstackVec3s(delta));
}

void GradientDescentSolver::reset()
{
    // no-op
}