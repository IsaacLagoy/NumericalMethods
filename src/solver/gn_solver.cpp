#include "solver.h"

void GaussNewtonSolver::step()
{
    const Eigen::VectorXf residual = computeResidual();
    if (residual.norm() < COST_EPSILON) return;

    const Eigen::MatrixXf jacobian = currJacobian();
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