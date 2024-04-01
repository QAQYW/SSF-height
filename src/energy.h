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

double calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj);
double calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj, std::vector<double> &speedSche);

} // namespace energy_calculator



#endif