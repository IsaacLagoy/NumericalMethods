#include "gn_solver.h"
#include "../skeleton/robot.h"

namespace {
    constexpr float DAMPING = 0.01f;
    constexpr float COST_EPSILON = 1e-6f;
}

void GaussNewtonSolver::step()
{
    const Eigen::VectorXf residual = currResidual();
    if (residual.norm() < COST_EPSILON)
    {
        return;
    }

    const Eigen::MatrixXf jacobian = currJacobian();
    const int dof = jacobian.cols();

    // Damped Gauss-Newton: (J^T J + λI) Δr = J^T e
    const Eigen::MatrixXf jtJ = jacobian.transpose() * jacobian;
    const Eigen::MatrixXf system = jtJ + DAMPING * Eigen::MatrixXf::Identity(dof, dof);
    const Eigen::VectorXf deltaR = system.ldlt().solve(jacobian.transpose() * residual);

    std::vector<glm::vec3> deltas(static_cast<size_t>(dof / 3));
    for (int j = 0; j < dof / 3; ++j)
    {
        deltas[static_cast<size_t>(j)] = glm::vec3(
            deltaR(3 * j + 0),
            deltaR(3 * j + 1),
            deltaR(3 * j + 2)
        );
    }

    robot->applyJointDeltas(deltas);
}
