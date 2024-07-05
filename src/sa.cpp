#include "sa.h"
#include "resource.h"
#include "tools.h"
#include "energy.h"
#include "problemDisc2D.h"
// #include "problemDisc1D.h"

/* ------------------------------- parameters ------------------------------- */

const double sa::INIT_TEMPERATURE = 1e8;
const double sa::MIN_TEMPERATURE = 1e-10;
const double sa::TEMPERATURE_REDUCE_COEF = 0.99;

/* -------------------------------- Solution -------------------------------- */

sa::Solution::Solution(int heightDiscNum, int lengthDiscNum, int flyHeightIndex) : heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum) {
    trajectory = Trajectory(lengthDiscNum, flyHeightIndex);
    position.resize(lengthDiscNum);
    for (int i = 0; i < lengthDiscNum; i++) {
        position[i] = flyHeightIndex + 0.5;
    }
}

sa::Solution::Solution(int heightDiscNum, int lengthDiscNum) : heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum) {
    position.resize(lengthDiscNum);
    for (int i = 0; i < lengthDiscNum; i++) {
        position[i] = tools::randDouble(0, heightDiscNum - 0.0001);
    }
    trajectory = Trajectory(lengthDiscNum);
    positionToTrajectory();
}

double sa::Solution::getCost() const {
    return cost;
}

void sa::Solution::positionToTrajectory() {
    for (int i = 0; i < lengthDiscNum; i++) {
        trajectory.setHeightIndex(i, (int) position[i]);
    }
}

Trajectory sa::Solution::getTrajectory() const {
    return trajectory;
}

double sa::Solution::calHeightCost() const {
    return trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR);
}

double sa::Solution::calSpeedCost(const ProblemDisc2D &problem) const {
    return energy_calculator::calSpeedCost(problem, trajectory);
}

void sa::Solution::calCost(const ProblemDisc2D &problem) {
    hcost = calHeightCost();
    vcost = calSpeedCost(problem);
    cost = hcost + vcost;
}

void sa::Solution::update(double temperature, const sa::SASolver &solver) {
    std::vector<double> origin(lengthDiscNum);
    double originCost = cost;
    for (int i = 0; i < lengthDiscNum; i++) origin[i] = position[i];
    int count = 100;
    while (count--) {
        for (int i = 0; i < lengthDiscNum; i++) {
            position[i] = origin[i] + (tools::randDouble(0, heightDiscNum - 0.0001) * 2 - (heightDiscNum - 0.0001)) * temperature;
            if (position[i] > heightDiscNum - 0.0001) {
                position[i] = heightDiscNum - 0.0001;
            } else if (position[i] < 0) {
                position[i] = 0;
            }
        }
        positionToTrajectory();
        if (solver.isFeasible(trajectory)) break;
    }
    if (count < 0) {
        for (int i = 0; i < lengthDiscNum; i++) position[i] = origin[i];
        return;
    }
    calCost(*solver.getProblem());
    if (cost <= originCost) {
        return;
    }
    if (std::exp((originCost - cost) / temperature) * RAND_MAX > std::rand()) {
        return;
    }
    // 不接受新解
    for (int i = 0; i < lengthDiscNum; i++) position[i] = origin[i];
}

/* -------------------------------- SASolver -------------------------------- */

sa::SASolver::SASolver(ProblemDisc2D* prob) : problem(prob) {
    heightDiscNum = prob->getHeightDiscNum();
    lengthDiscNum = prob->getLengthDiscNum();
    bestSolution = sa::Solution(heightDiscNum, lengthDiscNum, 0);
    bestSolution.calCost(*prob);
}

ProblemDisc2D* sa::SASolver::getProblem() const {
    return problem;
}

void sa::SASolver::solve() {
    // bestSolution = sa::Solution(heightDiscNum, lengthDiscNum, 0);
    // bestSolution.calCost(*problem);

    sa::Solution solution = sa::Solution(heightDiscNum, lengthDiscNum);
    while (!isFeasible(solution.getTrajectory())) solution = sa::Solution(heightDiscNum, lengthDiscNum);
    solution.calCost(*problem);

    bestSolution = sa::Solution(solution);
    bestSolution.calCost(*problem);

    double temperature = sa::INIT_TEMPERATURE;
    while (temperature > sa::MIN_TEMPERATURE) {
        solution.update(temperature, *this);
        temperature *= sa::TEMPERATURE_REDUCE_COEF;
        if (solution.getCost() < bestSolution.getCost()) {
            bestSolution = sa::Solution(solution);
        }
    }
}

Trajectory sa::SASolver::getTrajectory() const {
    return bestSolution.getTrajectory();
}

bool sa::SASolver::isFeasible(Trajectory traj) const {
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