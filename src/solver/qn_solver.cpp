#include "solver.h"

QuasiNewtonSolver::QuasiNewtonSolver(Robot* robot) : Solver(robot) 
{
    reset();
}

void QuasiNewtonSolver::step()
{
    // compute search direction
    const Eigen::VectorXf gradient = computeResidual();
    const Eigen::VectorXf direction = -B_k_inv * gradient;

    // perform line search
    
}

void QuasiNewtonSolver::reset()
{
    x_k = computeResidual();
    const float dof = static_cast<float>(robot->getActuatedJoints().size());
    B_k_inv = Eigen::MatrixXf::Identity(dof, dof);
}