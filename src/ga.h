#ifndef GA_H
#define GA_H

#include <vector>

#include "trajectory.h"

// 前置声明
class ProblemDisc2D;

// 遗传算法
namespace ga {

extern int const MAX_ITERATOR; // 最大迭代次数
extern int const POPULATION_SIZE; // 种群规模，必须偶数
extern double const MUTATION_PROBABILITY; // 变异概率 0.005

// 个体
class Individual {
private:
    int heightDiscNum;
    int lengthDiscNum;
    double cost;
    Trajectory trajectory;

public:
    /// @brief 构造函数。随机生成一个trajectory
    /// @param heightDiscNum 离散高度值的数量
    /// @param lengthDiscNum 离散距离值的数量（=路径长度）
    Individual(int heightDiscNum, int lengthDiscNum);
    /// @brief 构造函数
    /// @param heightDiscNum 离散高度值的数量
    /// @param lengthDiscNum 离散距离值的数量（=路径长度）
    /// @param traj 无人机飞行路径
    Individual(int heightDiscNum, int lengthDiscNum, Trajectory traj);
    Trajectory getTrajectory() const;
    double getCost() const;
    void calCost(const ProblemDisc2D &problem);
    double calHeightCost() const;
    double calSpeedCost(const ProblemDisc2D &problem) const;
    /// @brief 变异操作。每个位置都有固定的概率变异为其他的飞行高度。
    void mutation();

    bool operator< (const Individual& _individual) const;
};

// 种群-个体Individual的集合
typedef std::vector<Individual> Population;


class GASolver {
private:
    ProblemDisc2D *problem;
    int heightDiscNum;
    int lengthDiscNum;
    Trajectory trajectory;
    double cost;

public:
    GASolver(ProblemDisc2D *prob);
    double getCost() const;
    Trajectory getTrajectory() const;
    void solve();
    void crossover(Population &children, Individual p1, Individual p2) const;
    void selection(Population &parents, const Population &children) const;
};


} // namespace ga




#endif