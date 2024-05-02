#include "greedy.h"
#include "energy.h"
#include "resource.h"
#include "problemDisc2D.h"

#include <cmath>

greedy::GreedySolver::GreedySolver(ProblemDisc2D *prob) : problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
    minHeightIndex = prob->getMinHeightIndex();
    maxHeightIndex = prob->getMaxHeightIndex();
}

Trajectory greedy::GreedySolver::getTrajectory() const {
    return trajectory;
}

void greedy::GreedySolver::solve() {
    int maxAvailableHeightIndex = maxHeightIndex;
    for (resource::SensorDisc2D sensor : problem->getSensorList()) {
        maxAvailableHeightIndex = std::min(maxAvailableHeightIndex, (int) sensor.rangeList.size() - 1);
    }

    trajectory = Trajectory(lengthIndexNum, minHeightIndex);
    minCost = trajectory.calHeightCost() + energy_calculator::calSpeedCost(*problem, trajectory);

    Trajectory tempTraj;
    double tempCost;
    for (int height = minHeightIndex + 1; height <= maxAvailableHeightIndex; height++) {
        tempTraj = Trajectory(lengthIndexNum, height);
        tempCost = tempTraj.calHeightCost() + energy_calculator::calSpeedCost(*problem, tempTraj);
        if (tempCost < minCost) {
            minCost = tempCost;
            trajectory = Trajectory(tempTraj);
        }
    }
}