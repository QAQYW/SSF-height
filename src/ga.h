#ifndef GA_H
#define GA_H

#include <vector>

#include "trajectory.h"

// 前置声明
class ProblemDisc2D;

// 遗传算法
namespace ga {

extern double const MUTATION_PROBABILITY; // 变异概率 0.005

// 个体
class Individual {
private:
    int heightDiscNum;
    int lengthDiscNum;
    double cost;
    Trajectory trajectory;

public:
    Individual();
    Trajectory getTrajectory() const;
    double getCost() const;
    void calCost(const ProblemDisc2D &problem);
    double calHeightCost() const;
    double calSpeedCost(const ProblemDisc2D &problem) const;
    /// @brief 变异操作。每个位置都有固定的概率变异为其他的飞行高度。
    void mutation();
};

// 种群-个体Individual的集合
typedef std::vector<Individual> Population;


class GASolver {
private:
    ProblemDisc2D *problem;

public:
    GASolver(ProblemDisc2D *prob);
};


} // namespace ga




#endif