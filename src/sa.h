#ifndef SA_H
#define SA_H

#include <vector>

#include "trajectory.h"

class ProblemDisc2D;

// 模拟退火算法，SA
namespace sa {

extern const double INIT_TEMPERATURE;
extern const double MIN_TEMPERATURE;
extern const double TEMPERATURE_REDUCE_COEF; // 0.995

class Solution {
private:
    int heightDiscNum;
    int lengthDiscNum;
    std::vector<double> position;
    Trajectory trajectory;
    double cost;
    double hcost;
    double vcost;

public:
    Solution() {};
    /// @brief 随机飞行轨迹
    /// @param heightDiscNum 
    /// @param lengthDiscNum 
    Solution(int heightDiscNum, int lengthDiscNum);
    /// @brief 指定飞行高度
    /// @param heightDiscNum 
    /// @param lengthDiscNum 
    /// @param flyHeightIndex 
    Solution(int heightDiscNum, int lengthDiscNum, int flyHeightIndex);
    // /// @brief 指定轨迹
    // /// @param heightDiscNum 
    // /// @param lengthDiscNum 
    // /// @param traj 
    // Solution(int heightDiscNum, int lengthDiscNum, const Trajectory &traj);
    double getCost() const;
    void positionToTrajectory();
    Trajectory getTrajectory() const;
    double calHeightCost() const;
    double calSpeedCost(const ProblemDisc2D &problem) const;
    void calCost(const ProblemDisc2D &problem);
    void update(double temperature, const SASolver &solver);
};

class SASolver {
private:
    ProblemDisc2D* problem;
    int heightDiscNum;
    int lengthDiscNum;
    Solution bestSolution;

public:
    SASolver(ProblemDisc2D* prob);
    ProblemDisc2D* getProblem() const;
    void solve();
    Trajectory getBestTrajectory() const;
    /// @brief 检查trajectory是否合法
    /// @param traj 
    /// @return 合法true；不合法false
    bool isFeasible(Trajectory traj) const;
};



}


#endif