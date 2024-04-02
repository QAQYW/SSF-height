#include "naive.h"
#include "energy.h"

bool naive::State::isSubset(const State &s) const {
    int sensorNum = sensorFlag.size();
    for (int i = 0; i < sensorNum; i++) {
        if (s.sensorFlag[i] && !sensorFlag[i]) {
            return false;
        }
    }
    return true;
}

naive::NaiveSolver::NaiveSolver(ProblemDisc2D *prob)
 : problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
    minHeightIndex = problem->getMinHeightIndex();
    maxHeightIndex = problem->getMaxHeightIndex();
}

int naive::NaiveSolver::getSensorNum() const {
    return sensorNum;
}

int naive::NaiveSolver::getLengthIndexNum() const {
    return lengthIndexNum;
}

int naive::NaiveSolver::getHeightIndexNum() const {
    return heightIndexNum;
}

Trajectory naive::NaiveSolver::getTrajectory() const {
    return trajectory;
}

void naive::NaiveSolver::solve() {
    // 以固定高度 minHeightIndex 飞行的轨迹，作为初始解
    trajectory = Trajectory(lengthIndexNum, minHeightIndex);
    // minCost = trajectory.calHeightCost() + trajectory.calSpeedCost(*problem);
    minCost = trajectory.calHeightCost() + energy_calculator::calSpeedCost(*problem, trajectory);

    Trajectory tempTraj(lengthIndexNum);
    for (int h = minHeightIndex; h <= maxHeightIndex; h++) {
        tempTraj.setHeightIndex(0, h);
        State start(0, h);
        checkSensors(0, h, start);
        generateTrajectory(start, tempTraj, lengthIndexNum);
    }
}

void naive::NaiveSolver::checkSensors(int lenId, int heiId, naive::State &state) const {
    state.sensorFlag.resize(sensorNum, false);
    for (int i = 0; i < sensorNum; i++) {
        state.sensorFlag[i] = problem->getSensor(i).isCovered(lenId, heiId);
    }
}

void naive::NaiveSolver::generateTrajectory(State curr, Trajectory &traj, int trajLen) {
    int l = curr.lenId + 1;

    if (l == trajLen) {
        // 先检查可行性（是否所有传感器的数据都能被收集）
        if (!isFeasible(traj)) return;
        // 计算能耗，更新最优路径
        // double cost = traj.calHeightCost() + traj.calSpeedCost(*problem);
        double cost = traj.calHeightCost() + energy_calculator::calSpeedCost(*problem, traj);
        if (cost < minCost) {
            minCost = cost;
            trajectory = traj;
        }
        return;
    }

    std::vector<State> candList; // candidate list of feasible states
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

bool naive::NaiveSolver::isFeasible(const Trajectory &traj) {
    int count = 0;
    bool vis[sensorNum];
    for (int d = 0; d < lengthIndexNum; d++) {
        int h = traj.getHeightIndex(d);
        for (int s = 0; s < sensorNum; s++) {
            if (vis[s]) continue;
            if (problem->getSensor(s).isCovered(d, h)) {
                vis[s] = true;
                ++count;
            }
        }
    }
    return (count == sensorNum);
}