#ifndef ACO_H
#define ACO_H

/* -------------------------------------------------------------------------- */
/*                            ACO (offline version)                           */
/* -------------------------------------------------------------------------- */

#include <cmath>
#include <iostream>
#include <vector>

#include "resource.h"
// #include "problemDisc1D.h"
// #include "problemDisc2D.h"
#include "ssf.h"
#include "tools.h"
#include "trajectory.h"
#include "energy.h"

// 前置声明
class ProblemDisc2D;

namespace aco {

/* -------------------------------- parameter ------------------------------- */

extern const int ANT_NUM;                       // 蚂蚁数量
extern const double ALPHA;                      // 信息素tau的指数
extern const double BETA;                       // 启发因子eta的指数
extern const double EVAPORATE_COEF;             // 信息素蒸发系数
extern const double ENHANCE_VALUE;              // 信息素增强值
extern const int MAX_INTERATOR;                 // 最大迭代次数
extern const double HEURISTIC_BASE;             // 启发式因子的基础值
extern const double HEURISTIC_REDUCE_FACTOR;    // 计算启发值时分母的系数
extern const double INITIAL_PHEROMONE_VALUE;    // 信息素初值

/* -------------------------------- roulette -------------------------------- */

/// @brief 轮盘赌中的待选项
struct Candidate {
    double p; // 概率
    int h;    // 高度
};

/// @brief 轮盘赌选择下一状态的高度
/// @param candList 待选项集合
/// @param sum 概率和（归一化前）
/// @return 被选中的待选项的高度（candidate.h）
int roulette(std::vector<aco::Candidate> &candList, double sum);

/* ----------------------------------- Ant ---------------------------------- */

class ACOSolver; // 前置声明

class Ant {
private:
    Trajectory trajectory;
    double cost;

public:
    Ant();
    // 初始化为以固定高度heightIndex飞行的轨迹，但不会自动计算cost
    Ant(int lengthDiscNum, int heightIndex);
    double getCost() const;
    void calCost(const ProblemDisc2D &problem);
    Trajectory getTrajectory() const;
    // 生成轨迹之前先初始化各变量（轨迹）
    void init(int lengthDiscNum);
    
    /// @brief 生成轨迹
    /// @param trajLen 轨迹长度
    /// @param ph 信息素矩阵
    /// @param solver ACO求解器
    void generateTrajectory(int trajLen, const std::vector<std::vector<std::vector<double>>> &ph, const aco::ACOSolver &solver);
    // 计算高度变化产生的能耗
    double calHeightCost() const;
    // 计算速度调度的能耗（不包含高度变化）
    double calSpeedCost(const ProblemDisc2D &problem) const;
};

/* -------------------------------- ACOSolver ------------------------------- */
class ACOSolver {
private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    Trajectory trajectory;
    // 辅助变量
    // std::vector<int> lBound; // 从0到当前位置d，经过的左边界数量
    std::vector<int> rBound; // 从0到当前位置d，经过的右边界数量

public:
    ACOSolver(ProblemDisc2D *prob);
    ProblemDisc2D* getProblem() const;
    Trajectory getTrajectory() const;
    int getSensorNum() const;
    void solve();
    // 信息素蒸发
    void evaporatePheromone(const std::vector<int>& dim, std::vector<std::vector<std::vector<double>>> &ph) const;
    // 用 bestAnt 的轨迹来增强信息素
    void enhancePheromone(const Ant &ant, std::vector<std::vector<std::vector<double>>> &ph) const;
    // 是否有快飞出范围了还没与无人机连接的传感器
    bool isUrgent(int d, std::vector<aco::Candidate> &candList, const std::vector<bool> &visit, int countVisit) const;
    // 计算被选中概率
    double calProbability(const std::vector<std::vector<std::vector<double>>> &ph, int d, int curr, int next) const;
    // 动态计算启发值
    double calHeuristic(int d, int curr, int next) const;

    /**
     * Online
    */
    
    // 在线问题下的使用这个函数代替 solve()
    // 将速度调度与传感器连接方案分别从 speedSche 与 linked 传出

    /// @brief 在线问题下的使用这个函数代替 solve()。将速度调度与传感器连接方案分别从 speedSche 与 linked 传出
    /// @param start 
    /// @param end 
    /// @param speedSche 速度调度
    /// @param linked 传感器连接方案
    void solveForOnline(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked);
};

} // namespace aco



#endif