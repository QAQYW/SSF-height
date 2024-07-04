#include "greedy2.h"
#include "energy.h"
#include "resource.h"
#include "problemDisc2D.h"

#include <cmath>

greedy2::GreedySolver2::GreedySolver2(ProblemDisc2D *prob): problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthDiscNum = prob->getLengthDiscNum();
    heightDiscNum = prob->getHeightDiscNum();
    minHeightIndex = prob->getMinHeightIndex();
}

Trajectory greedy2::GreedySolver2::getTrajectory() const {
    return trajectory;
}

void greedy2::GreedySolver2::solve() {
    trajectory = Trajectory(lengthDiscNum, minHeightIndex);
    cost = trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR) + energy_calculator::calSpeedCost(*problem, trajectory);
}