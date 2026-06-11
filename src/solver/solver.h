#ifndef SOLVER_H
#define SOLVER_H

#include <Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

constexpr float MEASUREMENT_EPSILON = 1e-4f;

class Robot;

class Solver {
protected:
    Robot* robot;

public:
    Solver(Robot* robot);
    ~Solver() = default;

    virtual void step() = 0;

    float currCost() const;

    // length is the number of end effectors * 3
    Eigen::VectorXf currResidual() const;
    Eigen::MatrixXf currJacobian() const;
    Eigen::MatrixXf currHessianJTJ() const;

private:
    int rows() const;
    glm::vec3 axisVector(int axis) const;

    static Eigen::VectorXf stackEndEffectorPositions(const std::vector<glm::vec3>& positions);

    Eigen::VectorXf perturbColumn(
        int jointIndex,
        int axis,
        const Eigen::VectorXf& x0,
        const std::vector<glm::quat>& savedRotations
    ) const;
};

#endif
