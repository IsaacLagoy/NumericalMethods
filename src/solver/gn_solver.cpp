#include "solver.h"

void GaussNewtonSolver::step()
{
    if (computeCost() < COST_EPSILON) return;

    const Eigen::VectorXf residual = computeResidual();
    const Eigen::MatrixXf jacobian = computeJacobian();
    const int dof = jacobian.cols();

    // Damped Gauss-Newton: (J^T J + λI) p = -J^T r
    const Eigen::MatrixXf jtJ = jacobian.transpose() * jacobian;
    const Eigen::MatrixXf system = jtJ + DAMPING * Eigen::MatrixXf::Identity(dof, dof);
    const Eigen::VectorXf rhs = -jacobian.transpose() * residual;
    const Eigen::VectorXf deltaR = system.inverse() * rhs;

    robot->applyJointDeltas(unstackVec3s(deltaR));
}

void GaussNewtonSolver::reset()
{
    // no-op
}