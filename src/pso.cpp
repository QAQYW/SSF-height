#include "pso.h"

#include <cmath>

#include "tools.h"
#include "problemDisc2D.h"
#include "problemDisc1D.h"
#include "energy.h"

/* -------------------------------- Partical -------------------------------- */

pso::Partical::Partical(int heightDiscNum, int lengthDiscNum, double gap, const ProblemDisc2D &problem)
: heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum), gap(gap) {
    // 随机初始化速度和位置
    speed.resize(lengthDiscNum);
    position.resize(lengthDiscNum);
    bestPosition.resize(lengthDiscNum);
    for (int i = 0; i < lengthDiscNum; i++) {
        speed[i] = tools::randDouble(0, 1);
        bestPosition[i] = position[i] = tools::randDouble(0, 1);
    }
    positionToTrajectory();
    calCost(problem);
    bestCost = cost;

    trajectory = Trajectory(lengthDiscNum);
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
    return trajectory.calHeightCost();
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

pso::PSOSolver::PSOSolver(ProblemDisc2D* prob, int heightDiscNum, int lengthDiscNum)
: problem(prob), heightDiscNum (heightDiscNum), lengthDiscNum(lengthDiscNum) {
    
    swarm.resize(pso::SWARM_SIZE);

    bestPartical = pso::Partical();

    gap = 1.0 / (double) heightDiscNum;
}

void pso::PSOSolver::solve() {
    // 初始惯性权重
    double inertia = pso::INITIAL_INERTIA_VALUE;
    // 惯性权重每次的减小量（线性）
    double dInertia = (pso::INITIAL_INERTIA_VALUE - pso::END_INERTIA_VALUE) / pso::MAX_ITERATOR;
    
    // 初始化粒子群
    int bestIndex = 0;
    for (int i = 0; i < pso::SWARM_SIZE; i++) {
        swarm[i] = pso::Partical(heightDiscNum, lengthDiscNum, gap, *problem);
        // swarm[i].calCost(*problem);
        if (swarm[i].getCost() < swarm[bestIndex].getCost()) bestIndex = i;
    }
    bestPartical = pso::Partical(swarm[bestIndex]);
    
    int iter = 0;
    while (iter < pso::MAX_ITERATOR) {
        bestIndex = 0;
        for (int i = 0; i < pso::SWARM_SIZE; i++) {
            swarm[i].updatePosition(inertia, bestPartical); // 包含了速度的更新
            swarm[i].positionToTrajectory(); // 把连续的position映射为离散的trajectory
            swarm[i].calCost(*problem);
            swarm[i].updatePersonalBest();
            if (swarm[i].getCost() < swarm[bestIndex].getCost()) bestIndex = i;
        }

        // 更新全局最优粒子（主要是位置）
        if (swarm[bestIndex].getCost() < bestPartical.getCost()) {
            bestPartical = pso::Partical(swarm[bestIndex]);
        }

        iter++;
        inertia -= dInertia;
    }
}