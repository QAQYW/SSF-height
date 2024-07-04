#ifndef GREEDY2_H
#define GREEDY2_H

#include <vector>

#include "trajectory.h"

class ProblemDisc2D;

namespace greedy2 {

class GreedySolver2 {
private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthDiscNum;
    int heightDiscNum;
    int minHeightIndex;
    Trajectory trajectory;
    double cost;
public:
    GreedySolver2(ProblemDisc2D *prob);
    Trajectory getTrajectory() const;
    void solve();
};

}

#endif