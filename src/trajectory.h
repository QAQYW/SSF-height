#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "resource.h"
// #include "problemDisc1D.h"
// #include "ssf.h"

// 前置声明
// class ProblemDisc2D;

/// @brief 离散化后的轨迹（高度调度）
class Trajectory {

private:
    // 高度调度方案（每个离散的距离点对应的飞行高度）
    std::vector<int> heightSche;

public:
    Trajectory() {};
    /// @brief 初始化为以固定飞行高度（默认为最低高度）的轨迹
    /// @param size 轨迹长度
    Trajectory(int size);
    /// @brief 初始化为以固定飞行高度的轨迹
    /// @param size 轨迹长度
    /// @param heightIndex 飞行高度
    Trajectory(int size, int heightIndex);
    /// @brief 重新初始化为以固定飞行高度的轨迹
    /// @param size 轨迹长度
    /// @param heightIndex 飞行高度
    void reInit(int size, int heightIndex);
    void setHeightIndex(int lengthIndex, int heightIndex);
    void addList(int heightIndex);
    std::vector<int> getHeightSche() const;
    int getHeightIndex(int lengthIdnex) const;
    // 计算高度变化产生的能耗
    double calHeightCost(double coef) const;
    // // 计算速度调度的能耗（不包含高度变化）
    // double calSpeedCost(const ProblemDisc2D &problem2D) const;
    // // 计算速度调度的能耗（不包含高度变化），并将速度调度由参数speedSche传出
    // double calSpeedCost(const ProblemDisc2D &problem2D, std::vector<double> &speedSche) const;
};




#endif