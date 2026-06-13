#include "solver.h"

LevenbergMarquardtSolver::LevenbergMarquardtSolver(Robot* robot, float initMu) : 
    Solver(robot), 
    initMu(initMu), 
    mu(initMu)
{}

void LevenbergMarquardtSolver::step()
{
    const Eigen::VectorXf residual = computeResidual();
    if (residual.norm() < COST_EPSILON) return;

    // compute the variable delta
    const Eigen::MatrixXf jacobian = currJacobian();
    const Eigen::MatrixXf jtj = currHessianJTJ();
    const int dof = jacobian.cols();
    const Eigen::MatrixXf lhs = jtj + mu * Eigen::MatrixXf::Identity(dof, dof);
    const Eigen::VectorXf rhs = -jacobian.transpose() * residual;
    const Eigen::VectorXf p = lhs.inverse() * rhs;

    // predicted reduction in ||r||^2 for model ||r + Jp||^2 + mu||p||^2
    const float predicted = p.dot(jtj * p) + mu * p.squaredNorm();

    // compute gain ratio
    const float f_current = residual.squaredNorm();
    const float f_new = computeResidual(p).squaredNorm();
    const float gamma = (f_current - f_new) / predicted;
    
    if (gamma > 0.75) mu = mu / V;
    else if (gamma < 0.25) mu = mu * V;

    // accept the step
    if (gamma > 0.0)
    {
        robot->applyJointDeltas(unstackVec3s(p));
        return;
    }

    // reject the step
}

void LevenbergMarquardtSolver::reset()
{
    mu = initMu;
}