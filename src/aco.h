#ifndef ACO_H
#define ACO_H

#include <cmath>
#include <iostream>

#include "resource.h"
#include "problemDisc1D.h"
#include "problemDisc2D.h"
#include "ssf.h"
#include "tools.h"

class ProblemDisc2D;

namespace aco {

/* -------------------------------- parameter ------------------------------- */

extern const int ANT_NUM;                       // 蚂蚁数量
extern const double ALPHA;                      // 信息素tau的指数
extern const double BETA;                       // 启发因子eta的指数
extern const double INITIAL_PHEROMONE_VALUE;    // 信息素初值
extern const double EVAPORATE_COEF;             // 信息素蒸发系数
extern const double ENHANCE_VALUE;              // 信息素增强值
extern const int MAX_INTERATOR;                 // 最大迭代次数
extern const double HEURISTIC_BASE;             // 启发式因子的基础值
extern const double HEURISTIC_REDUCE_FACTOR;    // 计算启发值时分母的系数

/* -------------------------------- roulette -------------------------------- */

// 轮盘赌中的待选项
struct Candidate {
    double p; // 概率
    int h;    // 高度
};

// 返回被选中的candidate.h
int roulette(vector<aco::Candidate> &candList, double sum);

/* ------------------------------- Trajectory ------------------------------- */

/**
 * 只有离散化后的轨迹（高度调度），不涉及速度调度
*/
class Trajectory {
private:
    // 每个距离对应的高度
    vector<int> heightSche; // list of height index
public:
    Trajectory() {};
    // 初始化为以固定高度0飞行的轨迹
    Trajectory(int size);
    // 初始化为以固定高度heightIndex飞行的轨迹
    Trajectory(int size, int heightIndex);
    // 重新初始化
    void reInit(int size, int heightIndex);
    void setHeightIndex(int lengthIndex, int heightIndex);
    void addList(int heightIndex);
    vector<int> getHeightSche() const;
    int getHeightIndex(int lengthIdnex) const;
    // 计算高度变化产生的能耗
    double calHeightCost() const;
    // 计算速度调度的能耗（不包含高度变化）
    double calSpeedCost(const ProblemDisc2D &problem2D) const;
    // 计算速度调度的能耗（不包含高度变化），并获得解
    double calSpeedCost(const ProblemDisc2D &problem2D, vector<double> &speedSche) const;
};

/* ----------------------------------- Ant ---------------------------------- */
class ACOSolver; // 前置声明
class Ant {
private:
    Trajectory trajectory;
    double cost;

public:
    Ant(): trajectory() {};
    // 初始化为以固定高度heightIndex飞行的轨迹，但不会自动计算cost
    Ant(int lengthDiscNum, int heightIndex): trajectory(lengthDiscNum, heightIndex) {};
    // 析构函数，释放trajectory的内存
    // ~Ant();
    double getCost() const;
    void calCost(const ProblemDisc2D &problem);
    Trajectory getTrajectory() const;
    // 生成轨迹之前先初始化各变量
    void init(int lengthDiscNum);
    // 生成轨迹
    void generateTrajectory(int trajLen, const vector<vector<vector<double>>> &ph, const aco::ACOSolver &solver);
    // 计算高度变化产生的能耗
    double calHeightCost() const;
    // 计算速度调度的能耗（不包含高度变化）
    double calSpeedCost(const ProblemDisc2D &problem) const;
};

/* --------------------------------- solver --------------------------------- */
class ACOSolver {
private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    Trajectory trajectory;
    // 辅助变量
    // vector<int> lBound; // 从0到当前位置d，经过的左边界数量
    vector<int> rBound; // 从0到当前位置d，经过的右边界数量

public:
    ACOSolver(ProblemDisc2D *prob);
    // ~ACOSolver();
    ProblemDisc2D* getProblem() const;
    Trajectory getTrajectory() const;
    int getSensorNum() const;
    // int getLBoundValue(int index) const;
    int getRBoundValue(int index) const;
    void solve();
    // 信息素蒸发
    void evaporatePheromone(const vector<int>& dim, vector<vector<vector<double>>> &ph) const;
    // 用 bestAnt 的轨迹来增强信息素
    void enhancePheromone(const Ant &ant, vector<vector<vector<double>>> &ph) const;
    // 是否有快飞出范围了还没与无人机连接的传感器
    bool isUrgent(int d, vector<aco::Candidate> &candList, const vector<bool> &visit, int countVisit) const;
    // 计算被选中概率
    double calProbability(const vector<vector<vector<double>>> &ph, int d, int curr, int next) const;
    // 动态计算启发值
    double calHeuristic(int d, int curr, int next) const;
    // 复制轨迹
    // void copyTrajectory(const aco::Trajectory &traj);
};

}
// namespace aco 

#endif