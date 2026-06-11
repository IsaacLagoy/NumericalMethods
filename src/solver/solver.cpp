#include "solver.h"
#include "../skeleton/robot.h"
#include "../skeleton/bone.h"

#include <stdexcept>

Solver::Solver(Robot* robot) : robot(robot) {}

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

int Solver::rows() const
{
    // task space has xyz for each end-effector (4 EEs -> 12 rows).
    return robot->getEndEffectorCount() * 3;
}

glm::vec3 Solver::axisVector(int axis) const
{
    switch (axis)
    {
        case 0: return glm::vec3(1.0f, 0.0f, 0.0f);
        case 1: return glm::vec3(0.0f, 1.0f, 0.0f);
        case 2: return glm::vec3(0.0f, 0.0f, 1.0f);
        default: throw std::invalid_argument("Invalid axis");
    }
}

Eigen::VectorXf Solver::stackEndEffectorPositions(const std::vector<glm::vec3>& positions)
{
    // flatten [vec3, vec3, ...] into [x0,y0,z0, x1,y1,z1, ...] for Eigen math.
    Eigen::VectorXf stacked(static_cast<Eigen::Index>(positions.size() * 3));
    for (size_t i = 0; i < positions.size(); ++i)
    {
        stacked(3 * static_cast<Eigen::Index>(i) + 0) = positions[i].x;
        stacked(3 * static_cast<Eigen::Index>(i) + 1) = positions[i].y;
        stacked(3 * static_cast<Eigen::Index>(i) + 2) = positions[i].z;
    }
    return stacked;
}

// ------------------------------------------------------------
// Set up
// ------------------------------------------------------------

float Solver::currCost() const
{
    // compute residual MSE
    const Eigen::VectorXf residual = currResidual();
    return residual.squaredNorm() / residual.size();
}

Eigen::VectorXf Solver::currResidual() const
{
    // one residual row block per end-effector (positive => EE should move toward target).
    const int rows = this->rows();
    Eigen::VectorXf residual = Eigen::VectorXf::Zero(rows);

    // read current FK positions and fixed world-space targets (zipped by index).
    const std::vector<glm::vec3> endEffectorPositions = robot->getEndEffectorPositions();
    const std::vector<glm::vec3> targetPositions = robot->getTargetPositions();
    for (int i = 0; i < rows; i += 3)
    {
        const int ee = i / 3;
        residual(i + 0) = targetPositions[ee].x - endEffectorPositions[ee].x;
        residual(i + 1) = targetPositions[ee].y - endEffectorPositions[ee].y;
        residual(i + 2) = targetPositions[ee].z - endEffectorPositions[ee].z;
    }

    return residual;
}

Eigen::MatrixXf Solver::currJacobian() const
{
    // J: how each local joint tangent axis moves all EE world positions.
    const auto& actuatedJoints = robot->getActuatedJoints();
    const int rows = this->rows();
    const int cols = static_cast<int>(actuatedJoints.size() * 3);

    // save pose once so finite-difference columns can restore without copying the robot
    // copied once here instead of in the preturb function
    std::vector<glm::quat> savedRotations;
    savedRotations.reserve(actuatedJoints.size());
    for (const Bone* joint : actuatedJoints)
    {
        savedRotations.push_back(joint->getRot());
    }

    // baseline stacked EE positions at the current (unperturbed) configuration.
    const Eigen::VectorXf x0 = stackEndEffectorPositions(robot->getEndEffectorPositions());

    // fill one column per (joint, local axis) via forward finite differencing.
    Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(rows, cols);
    for (size_t j = 0; j < actuatedJoints.size(); ++j)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            jacobian.col(static_cast<Eigen::Index>(3 * j + axis)) = perturbColumn(
                static_cast<int>(j), axis, x0, savedRotations
            );
        }
    }

    return jacobian;
}

// ------------------------------------------------------------
// Perturbation
// ------------------------------------------------------------

Eigen::VectorXf Solver::perturbColumn(
    int jointIndex,
    int axis,
    const Eigen::VectorXf& x0,
    const std::vector<glm::quat>& savedRotations) const
{
    // nudge one joint along one local tangent direction by a small angle epsilon.
    robot->applyJointDelta(jointIndex, axisVector(axis) * MEASUREMENT_EPSILON);

    // measure how all end-effector positions changed after the nudge.
    const Eigen::VectorXf x1 = stackEndEffectorPositions(robot->getEndEffectorPositions());

    // put the skeleton back so the caller's robot state is unchanged after Jacobian build.
    auto& actuatedJoints = robot->getActuatedJoints();
    for (size_t i = 0; i < savedRotations.size(); ++i)
    {
        actuatedJoints[i]->setRot(savedRotations[i]);
    }

    // finite-difference estimate of d x_ee / d delta_r for this column.
    return (x1 - x0) / MEASUREMENT_EPSILON;
}
