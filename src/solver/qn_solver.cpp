#include "qn_solver.h"

QuasiNewtonSolver::QuasiNewtonSolver(Robot* robot, float c1, float c2) : WolfeLineSearch(robot, c1, c2) 
{
    reset();
}

void QuasiNewtonSolver::step()
{
    if (computeCost() < COST_EPSILON) return;

    // compute search direction
    const Eigen::VectorXf gradient = computeGradient();
    const Eigen::VectorXf direction = -BkInv * gradient;

    // perform line search
    const float fOld = computeCost();
    const float alpha = search(fOld, direction); // wolfe guarantees y_k^T s_k > 0

    if (alpha < 1e-6 || std::isnan(alpha)) return;

    // update x
    const Eigen::VectorXf jointDelta = alpha * direction; // s_k
    robot->applyJointDeltas(unstackVec3s(jointDelta));

    // compute gradient change
    const Eigen::VectorXf gradientNew = computeGradient();
    const Eigen::VectorXf y = gradientNew - gradient;

    const float denominator = y.transpose() * jointDelta;

    if (denominator <= 0.0f) return;

    // update inverse hessian approximation
    const float p = 1.0f / denominator;
    const int dof = static_cast<int>(3 * robot->getActuatedJointCount());
    BkInv = (Eigen::MatrixXf::Identity(dof, dof) - p * jointDelta * y.transpose()) * BkInv * (Eigen::MatrixXf::Identity(dof, dof) - p * y * jointDelta.transpose()) + p * jointDelta * jointDelta.transpose();
}

void QuasiNewtonSolver::reset()
{
    const int dof = static_cast<int>(3 * robot->getActuatedJointCount());
    BkInv = Eigen::MatrixXf::Identity(dof, dof);
}