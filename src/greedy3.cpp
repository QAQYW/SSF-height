#include "greedy3.h"
#include "resource.h"
#include "problemDisc2D.h"
#include "energy.h"

#include <cmath>
#include <algorithm>

/* --------------------------------- Sensor --------------------------------- */

greedy3::Sensor::Sensor(int l, int r, int id, const resource::SensorDisc2D &s): leftMost(l), rightMost(r), sensorIndex(id) {
    heightIndex = 0;
    int index = 0;
    for (resource::RangeDisc rg : s.rangeList) {
        // leftMost = std::min(leftMost, rg.leftIndex);
        // rightMost = std::max(rightMost, rg.rightIndex);
        if (rg.leftIndex <= leftMost && rg.rightIndex >= rightMost) {
            leftMost = rg.leftIndex;
            rightMost = rg.rightIndex;
            heightIndex = index;
        }
        ++index;
    }
}

int greedy3::Sensor::getLeftMost() const {
    return leftMost;
}

int greedy3::Sensor::getRightMost() const {
    return rightMost;
}

int greedy3::Sensor::getSensorIndex() const {
    return sensorIndex;
}

int greedy3::Sensor::getHeightIndex() const {
    return heightIndex;
}

bool greedy3::Sensor::operator< (const Sensor& _sensor) const {
    if (leftMost == _sensor.leftMost) {
        return rightMost > _sensor.rightMost;
    }
    return leftMost < _sensor.leftMost;
}

/* ------------------------------ GreedySolver3 ----------------------------- */

greedy3::GreedySolver3::GreedySolver3(ProblemDisc2D *prob): problem(prob) {
    sensorNum = prob->getSensorNum();
    heightDiscNum = prob->getHeightDiscNum();
    lengthDiscNum = prob->getLengthDiscNum();
    minHeightIndex = prob->getMinHeightIndex();
    maxHeightIndex = prob->getMaxHeightIndex();
}

Trajectory greedy3::GreedySolver3::getTrajectory() const {
    return trajectory;
}

double greedy3::GreedySolver3::getHcost() const {
    return hcost;
}

double greedy3::GreedySolver3::getVcost() const {
    return vcost;
}

double greedy3::GreedySolver3::getCost() const {
    return cost;
}

std::vector<double> greedy3::GreedySolver3::getSpeedSche() const {
    return speedSche;
}

void greedy3::GreedySolver3::solve() {
    trajectory = Trajectory(lengthDiscNum, minHeightIndex);
    std::vector<greedy3::Sensor> sensors;
    for (int i = 0; i < sensorNum; i++) {
        greedy3::Sensor sensor = greedy3::Sensor(lengthDiscNum - 1, 0, i, problem->getSensor(i));
        sensors.push_back(sensor);
    }
    sort(sensors.begin(), sensors.end());
    // std::vector<double> speed(lengthDiscNum, resource::V_STAR);
    speedSche.resize(lengthDiscNum, resource::V_STAR);
    for (int i = 0, l, r, h; i < sensorNum; i++) {
        l = sensors[i].getLeftMost();
        r = sensors[i].getRightMost();
        h = sensors[i].getHeightIndex();
        if (i + 1 < sensorNum) r = std::min(r, sensors[i + 1].getLeftMost());
        double dis = resource::indexToDistance(l, r - 1, resource::REF_UNIT_LENGTH);
        double v = dis / problem->getSensor(i).time;
        for (int j = l; j < r; j++) {
            if (speedSche[j] != resource::V_STAR) {
                std::cout << "err: multiple setting\n";
            }
            speedSche[j] = v;
            trajectory.setHeightIndex(j, h);
        }
    }
    hcost = trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR);
    vcost = energy_calculator::calSpeedCost(speedSche);
    cost = hcost + vcost;
}