#ifndef GREEDY3_H
#define GREEDY3_H

#include "trajectory.h"
#include <vector>

class ProblemDisc2D;

namespace greedy3 {

/// @brief 传感器的基本信息，用于排序
class Sensor {
private:
    int leftMost;
    int rightMost;
    int sensorIndex;
    /// @brief 最宽处对应的高度(index)
    int heightIndex;
public:
    Sensor(int l, int r, int id, const resource::SensorDisc2D &s);
    int getLeftMost() const;
    int getRightMost() const;
    int getSensorIndex() const;
    int getHeightIndex() const;
    /// @brief 先按照leftMost升序；若相同，再按照rightIndex降序
    /// @param _sensor 
    /// @return 
    bool operator< (const Sensor& _sensor) const;
};

class GreedySolver3 {
private:
    ProblemDisc2D* problem;
    Trajectory trajectory;
    int sensorNum;
    int heightDiscNum;
    int lengthDiscNum;
    int minHeightIndex;
    int maxHeightIndex;
    double cost;
    double hcost;
    double vcost;
    std::vector<double> speedSche;

public:
    GreedySolver3(ProblemDisc2D *prob);
    Trajectory getTrajectory() const;
    double getHcost() const;
    double getVcost() const;
    double getCost() const;
    std::vector<double> getSpeedSche() const;
    void solve();
};

}

#endif