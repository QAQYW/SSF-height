#include "pso.h"

#include <cmath>

#include "tools.h"
#include "problemDisc2D.h"
#include "problemDisc1D.h"
#include "energy.h"

/* -------------------------------- Parameter ------------------------------- */

const double pso::INITIAL_INERTIA_VALUE = 0.9;  // ! 未调参
const double pso::END_INERTIA_VALUE = 0.4;  // ! 未调参
const double pso::PERSONAL_BEST_COEF = 2.0;  // ! 未调参
const double pso::GLOBAL_BEST_COEF = 2.0;  // ! 未调参
const int pso::SWARM_SIZE = 20; //30;  // ! 未调参
const double pso::MAX_SPEED = 0.1;  // ! 未调参
const int pso::MAX_ITERATOR = 50; //30; //50; // 50;  // ! 未调参

/* -------------------------------- Partical -------------------------------- */

pso::Partical::Partical(int heightDiscNum, int lengthDiscNum, double gap, const ProblemDisc2D &problem)
: heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum), gap(gap) {
    // 初始化trajectory
    trajectory = Trajectory(lengthDiscNum);
    // 随机初始化速度和位置
    speed.resize(lengthDiscNum);
    position.resize(lengthDiscNum);
    bestPosition.resize(lengthDiscNum);
    for (int i = 0; i < lengthDiscNum; i++) {
        speed[i] = tools::randDouble(-pso::MAX_SPEED, pso::MAX_SPEED);
        bestPosition[i] = position[i] = tools::randDouble(0, 1);
    }
    // 计算cost
    positionToTrajectory();
    calCost(problem);
    bestCost = cost;
}

pso::Partical::Partical(int heightDiscNum, int lengthDiscNum, double gap, const ProblemDisc2D &problem, int heightIndex)
: heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum), gap(gap) {
    // 初始化trajectory
    trajectory = Trajectory(lengthDiscNum);
    // 随机初始化速度和位置（但都对应最低高度）
    speed.resize(lengthDiscNum);
    position.resize(lengthDiscNum);
    bestPosition.resize(lengthDiscNum);
    for (int i = 0; i < lengthDiscNum; i++) {
        speed[i] = tools::randDouble(-pso::MAX_SPEED, pso::MAX_SPEED);
        bestPosition[i] = position[i] = gap / 2;
    }
    // 计算cost
    positionToTrajectory();
    calCost(problem);
    bestCost = cost;
}

void pso::Partical::positionToTrajectory() {
    int hid;
    int hMax = heightDiscNum - 1;
    for (int i = 0; i < lengthDiscNum; i++) {
        hid = std::min(hMax, (int) (position[i] / gap));
        trajectory.setHeightIndex(i, hid);
    }
}

void pso::Partical::bestPositionToTrajectory() {
    int hid;
    int hMax = heightDiscNum - 1;
    for (int i = 0; i < lengthDiscNum; i++) {
        hid = std::min(hMax, (int) (bestPosition[i] / gap));
        trajectory.setHeightIndex(i, hid);
    }
}

std::vector<double> pso::Partical::getPosition() const {
    return position;
}

std::vector<double> pso::Partical::getBestPosition() const {
    return bestPosition;
}

double pso::Partical::getCost() const {
    return cost;
}

double pso::Partical::getBestCost() const {
    return bestCost;
}

Trajectory pso::Partical::getTrajectory() const {
    return trajectory;
}

Trajectory pso::Partical::getBestTrajectory() {
    bestPositionToTrajectory();
    return trajectory;
}

void pso::Partical::updatePosition(double inertia, const Partical &gb) {
    std::vector<double> gbp = gb.getBestPosition();
    double rd1 = tools::randDouble(0, 1);
    double rd2 = tools::randDouble(0, 1);
    for (int i = 0; i < lengthDiscNum; i++) {
        speed[i] = inertia * speed[i] + pso::PERSONAL_BEST_COEF * rd1 * (bestPosition[i] - position[i]) + pso::GLOBAL_BEST_COEF * rd2 * (gbp[i] - position[i]);
        if (speed[i] > pso::MAX_SPEED) speed[i] = pso::MAX_SPEED;
        if (speed[i] < (-pso::MAX_SPEED)) speed[i] = -pso::MAX_SPEED;
        position[i] += speed[i];
        if (position[i] > 1) position[i] = 1;
        if (position[i] < 0) position[i] = 0;
    }
}

void pso::Partical::calCost(const ProblemDisc2D &problem) {
    hcost = calHeightCost();
    vcost = calSpeedCost(problem);
    cost = hcost + vcost;
    // cost = calHeightCost() + calSpeedCost(problem);
}

double pso::Partical::calHeightCost() const {
    return trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR);
}

double pso::Partical::calSpeedCost(const ProblemDisc2D &problem) const {
    return energy_calculator::calSpeedCost(problem, trajectory);
}

void pso::Partical::updatePersonalBest() {
    if (cost < bestCost) {
        for (int i = 0; i < lengthDiscNum; i++) {
            bestPosition[i] = position[i];
        }
        bestCost = cost;
    }
}

/* -------------------------------- PSOSolver ------------------------------- */

pso::PSOSolver::PSOSolver(ProblemDisc2D* prob) : problem(prob) {
    heightDiscNum = prob->getHeightDiscNum();
    lengthDiscNum = prob->getLengthDiscNum();
    
    swarm.resize(pso::SWARM_SIZE);

    bestPartical = pso::Partical();

    gap = 1.0 / (double) heightDiscNum;
}

Trajectory pso::PSOSolver::getTrajectory() {
    return bestPartical.getBestTrajectory();
}

void pso::PSOSolver::solve() {
    // 初始惯性权重
    double inertia = pso::INITIAL_INERTIA_VALUE;
    // 惯性权重每次的减小量（线性）
    double dInertia = (pso::INITIAL_INERTIA_VALUE - pso::END_INERTIA_VALUE) / pso::MAX_ITERATOR;

    // // 初始化初始最优解（沿最低高度飞）
    // bestPartical = pso::Partical(heightDiscNum, lengthDiscNum, gap, *problem, 0);
    
    // 初始化粒子群
    int bestIndex = -1;
    while (bestIndex == -1) {
        for (int i = 0; i < pso::SWARM_SIZE; i++) {
            swarm[i] = pso::Partical(heightDiscNum, lengthDiscNum, gap, *problem);
            if (!isFeasible(swarm[i].getTrajectory())) continue;
            // swarm[i].calCost(*problem);
            if (bestIndex == -1 || swarm[i].getCost() < swarm[bestIndex].getCost()) bestIndex = i;
        }
        // if (~bestIndex) {
        //     bestPartical = pso::Partical(swarm[bestIndex]);
        // }
    }
    bestPartical = pso::Partical(swarm[bestIndex]);

    // 记录每次迭代后的optimal cost
    std::vector<double> optimalCostList;
    optimalCostList.push_back(bestPartical.getBestCost());
    
    int iter = 0;
    while (iter < pso::MAX_ITERATOR) {
        bestIndex = -1;
        bool flag = false;
        for (int i = 0; i < pso::SWARM_SIZE; i++) {
            swarm[i].updatePosition(inertia, bestPartical); // 包含了速度的更新
            swarm[i].positionToTrajectory(); // 把连续的position映射为离散的trajectory
            if (!isFeasible(swarm[i].getTrajectory())) {
                // std::cout << "PSO: Infeasible solution!\n";
                continue;
            }
            flag = true;
            swarm[i].calCost(*problem);
            swarm[i].updatePersonalBest();
            if (bestIndex == -1 || swarm[i].getCost() < swarm[bestIndex].getCost()) bestIndex = i;
        }

        if (!flag) continue;
        // 更新全局最优粒子（主要是位置）
        if (swarm[bestIndex].getCost() < bestPartical.getCost()) {
            bestPartical = pso::Partical(swarm[bestIndex]);
        }

        iter++;
        inertia -= dInertia;

        // 记录optimal cost
        optimalCostList.push_back(bestPartical.getBestCost());
    }

    // 输出 optimalCostList 到文件
    tools::printVector("PSO", ".\\newnewexp\\exp_iter\\iter_results.txt", optimalCostList);
}

bool pso::PSOSolver::isFeasible(Trajectory traj) const {
    int num = problem->getSensorNum();
    bool collected[num] = {false};
    for (int sid = 0; sid < num; sid++) {
        int lmost = lengthDiscNum - 1;
        int rmost = 0;
        for (resource::RangeDisc rg : problem->getSensor(sid).rangeList) {
            lmost = std::min(lmost, rg.leftIndex);
            rmost = std::max(rmost, rg.rightIndex);
        }
        for (int dis = lmost, hei; !collected[sid] && dis < rmost; dis++) {
            hei = traj.getHeightIndex(dis);
            if (problem->getSensor(sid).isCovered(dis, hei)) {
                collected[sid] = true;
            }
        }
        if (!collected[sid]) {
            return false;
        }
    }
    return true;
}