#include "line_search.h"

ArmijoLineSearch::ArmijoLineSearch(Robot* robot, float c1) : 
    LineSearch(robot), 
    c1(c1) 
{}

void ArmijoLineSearch::step()
{
    // no-op
}

void ArmijoLineSearch::reset()
{
    // no-op
}

float ArmijoLineSearch::search(float fOld, const Eigen::VectorXf& direction)
{
    // backtrack until we find a valid alpha
    float alpha = 1.0f;
    float slope = direction.dot(computeGradient());
    
    for (int i = 0; i < ARMIJO_MAX_ITERATIONS; i++) 
    {
        const float fNew = computeCost(alpha * direction);
        if (fNew <= fOld + c1 * alpha * slope) 
        {
            return alpha;
        }
        alpha *= ARMIJO_GAMMA;
    }

    return alpha;
}