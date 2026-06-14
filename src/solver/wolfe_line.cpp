#include "line_search.h"

WolfeLineSearch::WolfeLineSearch(Robot* robot, float c1, float c2) : 
    ArmijoLineSearch(robot, c1), 
    c2(c2) 
{}

void WolfeLineSearch::step()
{
    // no-op
}

void WolfeLineSearch::reset()
{
    // no-op
}

float WolfeLineSearch::search(float fOld, const Eigen::VectorXf& direction)
{
    float alphaLo, alphaHi;
    bool success = bracket(fOld, direction, alphaLo, alphaHi);

    // bracket phase found a valid alpha
    if (success) return alphaHi;

    return zoom(fOld, direction, alphaLo, alphaHi);
}

// ------------------------------------------------------------
// Private methods
// ------------------------------------------------------------

bool WolfeLineSearch::bracket(float fOld, const Eigen::VectorXf& direction, float& alphaLo, float& alphaHi)
{
    // walk forward from alpha=0, growing alpha until we can bracket a valid point
    float alphaPrev = 0.0f;
    float alpha = 1.0f;
    float fPrev = fOld;
    float slope = direction.dot(computeGradient());

    for (int i = 0; i < ARMIJO_MAX_ITERATIONS; i++)
    {
        float fNew = computeCost(alpha * direction);
        float slopeNew = direction.dot(computeGradient(alpha * direction));

        // not Armijo condition
        if (fNew > fOld + c1 * alpha * slope || (i > 0 && fNew > fPrev)) 
        {
            alphaLo = alphaPrev;
            alphaHi = alpha;
            return false;
        }

        // Wolfe condition
        if (slopeNew >= c2 * slope) 
        {
            alphaLo = alphaPrev;
            alphaHi = alpha;
            return true;
        }

        // slope went non-negative — minimum is behind us, bracket [alpha_prev, alpha]
        if (slopeNew >= 0)
        {
            alphaLo = alphaPrev;
            alphaHi = alpha;
            return false;
        }

        fPrev = fNew;
        alphaPrev = alpha;
        // alpha = std::min(2 * alpha, 1.0f);
        alpha = 2 * alpha;
    }

    // we did not find a bracket or alpha
    alphaLo = alphaPrev;
    alphaHi = alpha;
    return false;
}

float WolfeLineSearch::zoom(float fOld, const Eigen::VectorXf& direction, float alphaLo, float alphaHi)
{
    float fLo = computeCost(alphaLo * direction);
    float slopeLo = direction.dot(computeGradient(alphaLo * direction));
    float fHi = computeCost(alphaHi * direction);
    float slopeHi = direction.dot(computeGradient(alphaHi * direction));

    float slope = direction.dot(computeGradient());

    for (int i = 0; i < ARMIJO_MAX_ITERATIONS; i++)
    {
        float alphaTrial = cubic_interpolation(alphaLo, alphaHi, fLo, fHi, slopeLo, slopeHi);
        float fTrial = computeCost(alphaTrial * direction);
        float slopeTrial = direction.dot(computeGradient(alphaTrial * direction));

        // not Armijo condition
        if (fTrial > fOld + c1 * alphaTrial * slope)
        {
            alphaHi = alphaTrial;
            fHi = fTrial;
            slopeHi = slopeTrial;
            continue;
        }

        // Wolfe condition
        if (slopeTrial >= c2 * slope)
        {
            return alphaTrial;
        }

        // armijo ok but curvature not — raise lower bound
        alphaLo = alphaTrial;
        fLo = fTrial;
        slopeLo = slopeTrial;
    }

    // fallback if zoom didn't fully converge
    return alphaLo;
}

float WolfeLineSearch::cubic_interpolation(float alphaLo, float alphaHi, float fLo, float fHi, float slopeLo, float slopeHi)
{
    float d = slopeLo + slopeHi - 3 * (fHi - fLo) / (alphaHi - alphaLo);
    float beta = std::sqrt(d * d - slopeLo * slopeHi);
    return alphaHi - (alphaHi - alphaLo) * (slopeHi + beta - d) / (slopeHi - slopeLo + 2 * beta);
}