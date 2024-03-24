#include "naive.h"

bool naive_solver::State::isSubset(const State &s) const {
    int sensorNum = sensorFlag.size();
    for (int i = 0; i < sensorNum; i++) {
        if (s.sensorFlag[i] && !sensorFlag[i]) {
            return false;
        }
    }
    return true;
}

naive_solver::NaiveSolver::NaiveSolver(ProblemDisc2D *prob)
 : problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
    minHeightIndex = problem->getMinHeightIndex();
    maxHeightIndex = problem->getMaxHeightIndex();
}

int naive_solver::NaiveSolver::getSensorNum() const {
    return sensorNum;
}

int naive_solver::NaiveSolver::getLengthIndexNum() const {
    return lengthIndexNum;
}

int naive_solver::NaiveSolver::getHeightIndexNum() const {
    return heightIndexNum;
}

void naive_solver::NaiveSolver::solve() {
    // 以固定高度 minHeightIndex 飞行的轨迹，作为初始解
    trajectory = aco::Trajectory(lengthIndexNum, minHeightIndex);
    minCost = trajectory.calHeightCost() + trajectory.calSpeedCost(*problem);

    aco::Trajectory tempTraj(lengthIndexNum);
    for (int h = minHeightIndex; h <= maxHeightIndex; h++) {
        tempTraj.setHeightIndex(0, h);
        State start(0, h);
        checkSensors(0, h, start);
        generateTrajectory(start, tempTraj, lengthIndexNum);
    }
}

void naive_solver::NaiveSolver::checkSensors(int lenId, int heiId, naive_solver::State &state) const {
    state.sensorFlag.resize(sensorNum, false);
    for (int i = 0; i < sensorNum; i++) {
        state.sensorFlag[i] = problem->getSensor(i).isCovered(lenId, heiId);
    }
}

void naive_solver::NaiveSolver::generateTrajectory(State curr, aco::Trajectory &traj, int trajLen) {
    int l = curr.lenId + 1;

    if (l == trajLen) {
        double cost = traj.calHeightCost() + traj.calSpeedCost(*problem);
        if (cost < minCost) {
            minCost = cost;
            trajectory = traj;
        }
        return;
    }

    vector<State> candList; // candidate list of feasible states
    for (int h = minHeightIndex; h <= maxHeightIndex; h++) {
        State s(l, h);
        checkSensors(l, h, s);
        if (h == curr.heiId) {
            candList.push_back(s);
        } else if (!curr.isSubset(s)) {
            candList.push_back(s);
        }
    }
    for (State next : candList) {
        traj.setHeightIndex(l, next.heiId);
        generateTrajectory(next, traj, trajLen);
    }
}
