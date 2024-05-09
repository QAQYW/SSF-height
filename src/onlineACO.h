#ifndef ONLINE_ACO_H
#define ONLINE_ACO_H

#include <vector>
#include "aco.h"
#include "resource.h"
#include "trajectory.h"
#include "problemOnlineDisc2D.h"

namespace online_aco {

class SensorState {
private:
    double time;
    bool active;

public:
    /// @brief 默认传感器不活跃
    /// @param t 传输时间
    SensorState(double t): time(t), active(false) {};

    double getTime() const;
    bool isActive() const;
    void reduceTime(double dTime);
    void setActive();
    void setInactive();
    void clearTime();
};

class OnlineACOSolver {
private:
    ProblemOnlineDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    Trajectory trajectory;
    double cost;
    double hcost;
    double vcost;
    std::vector<online_aco::SensorState> sensorStates;

public:
    OnlineACOSolver(ProblemOnlineDisc2D *prob);
    double getCost() const;
    double getHcost() const;
    double getVcost() const;
    std::vector<SensorState> getSensorStates() const;
    void collectData(const std::vector<int> &linked, double v);
    void updateEnergy(double v, int currh, int nexth);
    std::vector<int> exploreNewSensor(int currd, int currh, const std::vector<resource::SensorOnlineDisc2D> &sensorList, std::vector<bool> &informed, int &currEnd);
    void resolve(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked);
    void solve(std::vector<double> &speedSche);
};

} // namespace online_aco

#endif