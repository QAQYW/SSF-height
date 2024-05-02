#ifndef GREEDY_H
#define GREEDY_H


#include "resource.h"
#include "trajectory.h"

// 前置声明
class ProblemDisc2D;

namespace greedy {

class GreedySolver {
private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    int minHeightIndex;
    int maxHeightIndex;
    Trajectory trajectory;
    double minCost;

public:
    GreedySolver(ProblemDisc2D *prob);
    Trajectory getTrajectory() const;
    void solve();
};

} // namespace greedy



#endif