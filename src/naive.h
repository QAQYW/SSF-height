#ifndef NAIVE_H
#define NAIVE_H

#include "resource.h"
#include "problemDisc2D.h"
// #include "aco.h"
#include "trajectory.h"

namespace naive {

// 描述当前状态的结构体
struct State {
    int lenId;  // length index
    int heiId;  // height index
    std::vector<bool> sensorFlag; // 当前位置覆盖的传感器集合，true表示覆盖
    State(int l, int h): lenId(l), heiId(h) {};
    // s覆盖的传感器集合是否是this覆盖的传感器集合的子集
    bool isSubset(const State &s) const;
};

class NaiveSolver {
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
    NaiveSolver(ProblemDisc2D *prob);
    int getSensorNum() const;
    int getLengthIndexNum() const;
    int getHeightIndexNum() const;
    Trajectory getTrajectory() const;
    void solve();

private:
    void checkSensors(int lenId, int heiId, naive::State &state) const;
    void generateTrajectory(State curr, Trajectory &traj, int trajLen);
    bool isFeasible(const Trajectory &traj);
};

}

#endif