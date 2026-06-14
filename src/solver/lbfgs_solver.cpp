#include "qn_solver.h"

LBFGS::LBFGS(Robot* robot, float c1, float c2) : WolfeLineSearch(robot, c1, c2) 
{
    reset();
}

void LBFGS::step()
{
    if (computeCost() < COST_EPSILON) return;

    const Eigen::VectorXf gradient = computeGradient();

    // Two-loop L-BFGS recursion for r = H_k * grad f (Nocedal & Wright, Alg. 7.4)
    Eigen::VectorXf q = gradient;

    // First loop: i = k-1, ..., k-m
    for (size_t i = stores.size(); i > 0; --i) {
        Store& store = stores[i - 1];
        store.alpha = store.rho * store.s.dot(q);
        q -= store.alpha * store.y;
    }

    // H_0 = gamma_k * I,  gamma_k = (s_{k-1}^T y_{k-1}) / (y_{k-1}^T y_{k-1})
    float gamma = 1.0f;
    if (!stores.empty()) {
        const Store& latest = stores.back();
        gamma = latest.s.dot(latest.y) / latest.y.squaredNorm();
    }
    Eigen::VectorXf r = gamma * q;

    // Second loop: i = k-m, ..., k-1
    for (size_t i = 0; i < stores.size(); ++i) {
        const Store& store = stores[i];
        const float beta = store.rho * store.y.dot(r);
        r += store.s * (store.alpha - beta);
    }

    const Eigen::VectorXf direction = -r;

    const float fOld = computeCost();
    const float alpha = search(fOld, direction);

    if (alpha < 1e-6 || std::isnan(alpha)) return;

    const Eigen::VectorXf s = alpha * direction;
    robot->applyJointDeltas(unstackVec3s(s));

    const Eigen::VectorXf gradientNew = computeGradient();
    const Eigen::VectorXf y = gradientNew - gradient;
    const float yTs = y.dot(s);

    if (yTs <= 0.0f) return;

    enqueueStore(s, y, 1.0f / yTs);
}

void LBFGS::reset()
{
    stores.clear();
}

void LBFGS::enqueueStore(const Eigen::VectorXf& s, const Eigen::VectorXf& y, float rho)
{
    if (stores.size() >= LBFGS_MAX_STORES)
    {
        stores.erase(stores.begin());
    }
    stores.push_back({ s, y, rho, 0.0f });
}
