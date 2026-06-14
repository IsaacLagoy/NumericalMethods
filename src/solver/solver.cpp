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

Eigen::VectorXf Solver::stackVec3s(const std::vector<glm::vec3>& vec3s)
{
    // flatten [vec3, vec3, ...] into [x0,y0,z0, x1,y1,z1, ...] for Eigen math.
    Eigen::VectorXf stacked(static_cast<Eigen::Index>(vec3s.size() * 3));
    for (size_t i = 0; i < vec3s.size(); ++i)
    {
        stacked(3 * static_cast<Eigen::Index>(i) + 0) = vec3s[i].x;
        stacked(3 * static_cast<Eigen::Index>(i) + 1) = vec3s[i].y;
        stacked(3 * static_cast<Eigen::Index>(i) + 2) = vec3s[i].z;
    }
    return stacked;
}

std::vector<glm::vec3> Solver::unstackVec3s(const Eigen::VectorXf& stacked)
{
    const size_t count = static_cast<size_t>(stacked.size() / 3);
    std::vector<glm::vec3> vec3s(count);
    for (size_t i = 0; i < count; ++i)
    {
        vec3s[i] = glm::vec3(
            stacked(3 * static_cast<Eigen::Index>(i) + 0),
            stacked(3 * static_cast<Eigen::Index>(i) + 1),
            stacked(3 * static_cast<Eigen::Index>(i) + 2)
        );
    }
    return vec3s;
}

Eigen::VectorXf Solver::computeResidual(
    const std::vector<glm::vec3>& eePositions,
    const std::vector<glm::vec3>& targetPositions
) {
    // one residual row block per end-effector (positive => EE should move toward target).
    const int n = static_cast<int>(eePositions.size());
    Eigen::VectorXf residual(3 * n);
    for (int ee = 0; ee < n; ++ee)
    {
        residual(3 * ee + 0) = targetPositions[ee].x - eePositions[ee].x;
        residual(3 * ee + 1) = targetPositions[ee].y - eePositions[ee].y;
        residual(3 * ee + 2) = targetPositions[ee].z - eePositions[ee].z;
    }
    return residual;
}

std::vector<glm::quat> Solver::saveActuatedRotations() const
{
    const auto& actuatedJoints = robot->getActuatedJoints();
    std::vector<glm::quat> saved;
    saved.reserve(actuatedJoints.size());
    for (const Bone* joint : actuatedJoints)
    {
        saved.push_back(joint->getRot());
    }
    return saved;
}

void Solver::restoreActuatedRotations(const std::vector<glm::quat>& saved) const
{
    auto& actuatedJoints = robot->getActuatedJoints();
    for (size_t i = 0; i < saved.size(); ++i)
    {
        actuatedJoints[i]->setRot(saved[i]);
    }
}

// ------------------------------------------------------------
// Set up
// ------------------------------------------------------------

float Solver::computeCost() const
{
    const Eigen::VectorXf residual = computeResidual();
    return residual.squaredNorm() / residual.size();
}

float Solver::computeCost(const Eigen::VectorXf& jointDelta) const
{
    const Eigen::VectorXf residual = computeResidual(jointDelta);
    return residual.squaredNorm() / residual.size();
}

Eigen::VectorXf Solver::computeResidual() const
{
    return computeResidual(
        robot->getEndEffectorPositions(),
        robot->getTargetPositions()
    );
}

Eigen::VectorXf Solver::computeResidual(const Eigen::VectorXf& jointDelta) const
{
    if (jointDelta.size() == 0) throw std::invalid_argument("jointDelta is empty");

    const std::vector<glm::quat> saved = saveActuatedRotations();
    robot->applyJointDeltas(unstackVec3s(jointDelta));
    const Eigen::VectorXf residual = computeResidual(
        robot->getEndEffectorPositions(),
        robot->getTargetPositions()
    );
    restoreActuatedRotations(saved);
    return residual;
}

Eigen::MatrixXf Solver::computeJacobian() const
{
    // J: residual Jacobian dr/d(delta), where r = target - ee.
    const auto& actuatedJoints = robot->getActuatedJoints();
    const int rows = this->rows();
    const int cols = static_cast<int>(actuatedJoints.size() * 3);

    // save pose once so finite-difference columns can restore without copying the robot
    const std::vector<glm::quat> savedRotations = saveActuatedRotations();

    // baseline stacked EE positions at the current (unperturbed) configuration.
    const Eigen::VectorXf x0 = stackVec3s(robot->getEndEffectorPositions());

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

Eigen::MatrixXf Solver::computeJacobian(const Eigen::VectorXf& jointDelta) const
{
    if (jointDelta.size() == 0) throw std::invalid_argument("jointDelta is empty");

    const std::vector<glm::quat> saved = saveActuatedRotations();
    robot->applyJointDeltas(unstackVec3s(jointDelta));
    const Eigen::MatrixXf jacobian = computeJacobian();
    restoreActuatedRotations(saved);
    return jacobian;
}

Eigen::MatrixXf Solver::computeHessianJTJ() const
{
    const Eigen::MatrixXf jacobian = computeJacobian();
    return jacobian.transpose() * jacobian;
}

Eigen::MatrixXf Solver::computeHessianJTJ(const Eigen::VectorXf& jointDelta) const
{
    if (jointDelta.size() == 0) throw std::invalid_argument("jointDelta is empty");

    const Eigen::MatrixXf jacobian = computeJacobian(jointDelta);
    return jacobian.transpose() * jacobian;
}

Eigen::VectorXf Solver::computeGradient(
    const Eigen::MatrixXf& jacobian,
    const Eigen::VectorXf& residual
) {
    return (2.0f / static_cast<float>(residual.size())) * jacobian.transpose() * residual;
}

Eigen::VectorXf Solver::computeGradient() const
{
    return computeGradient(computeJacobian(), computeResidual());
}

Eigen::VectorXf Solver::computeGradient(const Eigen::VectorXf& jointDelta) const
{
    if (jointDelta.size() == 0)
    {
        return computeGradient();
    }

    return computeGradient(computeJacobian(jointDelta), computeResidual(jointDelta));
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
    const Eigen::VectorXf x1 = stackVec3s(robot->getEndEffectorPositions());

    // put the skeleton back so the caller's robot state is unchanged after Jacobian build.
    restoreActuatedRotations(savedRotations);

    // dr/d(delta) = -d(ee)/d(delta) because r = target - ee.
    return -(x1 - x0) / MEASUREMENT_EPSILON;
}
