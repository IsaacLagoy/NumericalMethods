#ifndef LINE_SEARCH_H
#define LINE_SEARCH_H

#include "solver.h"

namespace {
    constexpr int ARMIJO_MAX_ITERATIONS = 10;
    constexpr float ARMIJO_GAMMA = 0.5f;
}

// interface line search
class LineSearch : public Solver {
public:
    LineSearch(Robot* robot) : Solver(robot) {}

    virtual void step() = 0;
    virtual void reset() = 0;

    // returns the alpha of the line search
    virtual float search(float fOld, const Eigen::VectorXf& direction) = 0;
};

// Armijo line search
class ArmijoLineSearch : public LineSearch {
protected: 
    float c1;

public:
    ArmijoLineSearch(Robot* robot, float c1);

    virtual void step() override;
    virtual void reset() override;
    virtual float search(float fOld, const Eigen::VectorXf& direction) override;
};

// Wolfe line search
class WolfeLineSearch : public ArmijoLineSearch {
protected:
    float c2;

public:
    WolfeLineSearch(Robot* robot, float c1, float c2);

    virtual void step() override;
    virtual void reset() override;
    virtual float search(float fOld, const Eigen::VectorXf& direction) override;

private:
    bool bracket(float fOld, const Eigen::VectorXf& direction, float& alphaLo, float& alphaHi);
    float zoom(float fOld, const Eigen::VectorXf& direction, float alphaLo, float alphaHi);
    float cubic_interpolation(float alphaLo, float alphaHi, float fLo, float fHi, float slopeLo, float slopeHi);
};

#endif