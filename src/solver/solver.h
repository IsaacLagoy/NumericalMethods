#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "../skeleton/robot.h"
#include <vector>

namespace {
    constexpr float DAMPING = 0.01f;
    constexpr float COST_EPSILON = 1e-6f;
    constexpr float V = 10.0f;
    constexpr float MEASUREMENT_EPSILON = 1e-4f;
}

// ------------------------------------------------------------
// Interface
// ------------------------------------------------------------

class Solver {
protected:
    Robot* robot;

public:
    Solver(Robot* robot);
    ~Solver() = default;

    virtual void step() = 0;
    virtual void reset() = 0;

    float computeCost() const;
    float computeCost(const Eigen::VectorXf& jointDelta) const;

    // length is the number of end effectors * 3
    Eigen::VectorXf computeResidual() const;
    Eigen::VectorXf computeResidual(const Eigen::VectorXf& jointDelta) const;
    static Eigen::VectorXf computeResidual(
        const std::vector<glm::vec3>& eePositions, 
        const std::vector<glm::vec3>& targetPositions
    );

    // J = dr/d(delta); residual r = target - ee, so r(delta + p) ~= r + Jp
    Eigen::MatrixXf computeJacobian() const;
    Eigen::MatrixXf computeJacobian(const Eigen::VectorXf& jointDelta) const;

    Eigen::MatrixXf computeHessianJTJ() const;
    Eigen::MatrixXf computeHessianJTJ(const Eigen::VectorXf& jointDelta) const;

    Eigen::VectorXf computeGradient() const;
    Eigen::VectorXf computeGradient(const Eigen::VectorXf& jointDelta) const;
    static Eigen::VectorXf computeGradient(
        const Eigen::MatrixXf& jacobian,
        const Eigen::VectorXf& residual
    );

private:
    glm::vec3 axisVector(int axis) const;
    std::vector<glm::quat> saveActuatedRotations() const;
    void restoreActuatedRotations(const std::vector<glm::quat>& saved) const;

    Eigen::VectorXf perturbColumn(
        int jointIndex,
        int axis,
        const Eigen::VectorXf& x0,
        const std::vector<glm::quat>& savedRotations
    ) const;

protected:
    int rows() const;

    static Eigen::VectorXf stackVec3s(const std::vector<glm::vec3>& vec3s);
    static std::vector<glm::vec3> unstackVec3s(const Eigen::VectorXf& stacked);
};

// ------------------------------------------------------------
// Implementation
// ------------------------------------------------------------

// Gauss-Newton
class GaussNewtonSolver : public Solver {
public:
    GaussNewtonSolver(Robot* robot) : Solver(robot) {}
    void step() override;
    void reset() override;
};

// Levenberg-Marquardt
class LevenbergMarquardtSolver : public Solver {
private:
    float initMu;
    float mu;

public:
    LevenbergMarquardtSolver(Robot* robot, float initMu);
    void step() override;
    void reset() override;
};

// Gradient Descent
class GradientDescentSolver : public Solver {
private:
    float stepSize;

public:
    GradientDescentSolver(Robot* robot, float stepSize);
    void step() override;
    void reset() override;
};

// Quasi-Newton
class QuasiNewtonSolver : public Solver {
private:
    Eigen::VectorXf x_k;
    Eigen::MatrixXf B_k_inv;

public:
    QuasiNewtonSolver(Robot* robot);
    void step() override;
    void reset() override;
};

#endif
