#ifndef QN_SOLVER_H
#define QN_SOLVER_H

#include "line_search.h"

namespace {
    constexpr int LBFGS_MAX_STORES = 10;
}

// Quasi-Newton
class QuasiNewtonSolver : public WolfeLineSearch {
private:
    Eigen::MatrixXf BkInv;

public:
    QuasiNewtonSolver(Robot* robot, float c1, float c2);
    void step() override;
    void reset() override;
};

// L-BFGS
// even though our objective changes over time, it is smooth and should still work
class LBFGS : public WolfeLineSearch {
private:
    struct Store {
        Eigen::VectorXf s;
        Eigen::VectorXf y;
        float rho;
        float alpha;
    };

private:
    std::vector<Store> stores;

public:
    LBFGS(Robot* robot, float c1, float c2);
    void step() override;
    void reset() override;

private:
    void enqueueStore(const Eigen::VectorXf& s, const Eigen::VectorXf& y, float rho);
};

#endif