#ifndef PSO_H
#define PSO_H

#include <vector>

#include "trajectory.h"

// 前置声明
class ProblemDisc2D;

// 离散粒子群优化算法，BPSO
namespace pso {

// some parameters
extern const double INITIAL_INERTIA_VALUE;
extern const double END_INERTIA_VALUE;
extern const double PERSONAL_BEST_COEF;
extern const double GLOBAL_BEST_COEF;
extern const int SWARM_SIZE;
extern const double MAX_SPEED;
extern const int MAX_ITERATOR;

// extern std::vector<double> pso_MapTable;

/// @brief 粒子
class Partical {
private:
    int heightDiscNum;
    int lengthDiscNum;
    /// @brief 无人机能耗作为粒子适应度值, cost = hcost + vcost
    double cost;
    double hcost;
    double vcost;
    /// @brief 粒子速度
    std::vector<double> speed;
    /// @brief 粒子位置
    std::vector<double> position;
    /// @brief 粒子到过的最优位置
    std::vector<double> bestPosition;
    /// @brief 粒子到过的最优位置（bestPosition）的适应度值
    double bestCost;
    Trajectory trajectory;
    double gap;

public:
    Partical() {};
    /// @brief 随机初始化
    /// @param heightDiscNum 
    /// @param lengthDiscNum 
    /// @param gap 
    /// @param problem 
    Partical(int heightDiscNum, int lengthDiscNum, double gap, const ProblemDisc2D &problem);
    /// @brief 初始化为指定高度的轨迹（固定高度飞行）
    /// @param heightDiscNum 
    /// @param lengthDiscNum 
    /// @param gap 
    /// @param problem 
    /// @param heightIndex 飞行高度
    Partical(int heightDiscNum, int lengthDiscNum, double gap, const ProblemDisc2D &problem, int heightIndex);
    /// @brief 把位置position映射为离散的路径解trajectory
    void positionToTrajectory();
    /// @brief 把bestPosition映射为离散的路径解trajectory
    void bestPositionToTrajectory();
    std::vector<double> getPosition() const;
    std::vector<double> getBestPosition() const;
    double getCost() const;
    double getBestCost() const;
    Trajectory getTrajectory() const;
    Trajectory getBestTrajectory();
    /// @brief 更新speed和position
    /// @param gb 全局最优解
    void updatePosition(double inertia, const Partical &gb);
    void calCost(const ProblemDisc2D &problem);
    double calHeightCost() const;
    double calSpeedCost(const ProblemDisc2D &problem) const;
    /// @brief 更新个体最优位置
    void updatePersonalBest();
};

class PSOSolver {
private:
    ProblemDisc2D* problem;
    int heightDiscNum;
    int lengthDiscNum;
    std::vector<Partical> swarm;
    Partical bestPartical;
    /// @brief 用于把位置映射成trajectory的
    double gap;

public:
    PSOSolver(ProblemDisc2D* prob);
    Trajectory getTrajectory();
    void solve();
    bool isFeasible(Trajectory traj) const;
};

} // namespace pso


#endif