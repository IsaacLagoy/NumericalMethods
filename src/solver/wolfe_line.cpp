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
    
}