#ifndef ENERGY_H
#define ENERGY_H

/**
 * 用于计算能耗
*/

#include <vector>

// #include "trajectory.h"
#include "problemDisc1D.h"
#include "problemDisc2D.h"
#include "ssf.h"

// 前置声明
class Trajectory;
class ProblemDisc2D;

namespace energy_calculator {

    /// @brief 计算速度调度的能耗
    /// @param prob2D 离散化二维问题
    /// @param traj 飞行轨迹（高度调度）
    /// @return 能耗
    double calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj);
    /// @brief 计算速度调度的能耗，并传出速度调度的结果
    /// @param prob2D 离散化二维问题
    /// @param traj 飞行轨迹（高度调度）
    /// @param speedSche 保存速度调度的结果
    /// @return 能耗
    double calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj, std::vector<double> &speedSche);
    /// @brief 计算速度调度的能耗，速度调度方案已给出
    /// @param speedSche 速度调度方案
    /// @return 能耗
    double calSpeedCost(const std::vector<double> &speedSche);

} // namespace energy_calculator



#endif