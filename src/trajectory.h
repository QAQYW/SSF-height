#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "resource.h"
#include "ssf.h"
#include "problemDisc1D.h"

// 前置声明
class ProblemDisc2D;

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
    // 计算速度调度的能耗（不包含高度变化），并将速度调度由参数speedSche传出
    double calSpeedCost(const ProblemDisc2D &problem2D, vector<double> &speedSche) const;
};

#endif