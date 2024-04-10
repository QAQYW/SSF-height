#include "naive.h"
#include "energy.h"

bool naive::State::isSubset(const State &s) const {
    int sensorNum = sensorFlag.size();
    for (int i = 0; i < sensorNum; i++) {
        if (s.sensorFlag[i] && !this->sensorFlag[i]) {
            // s覆盖了传感器i，而this没有覆盖传感器i，则一定不是子集
            return false;
        }
    }
    return true;
}

naive::NaiveSolver::NaiveSolver(ProblemDisc2D *prob) : problem(prob) {
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
    // trajectory = Trajectory(lengthIndexNum, maxHeightIndex);
    // minCost = trajectory.calHeightCost() + trajectory.calSpeedCost(*problem);
    minCost = trajectory.calHeightCost() + energy_calculator::calSpeedCost(*problem, trajectory);

    Trajectory tempTraj(lengthIndexNum);
    for (int h = minHeightIndex; h <= maxHeightIndex; h++) {
        tempTraj.setHeightIndex(0, h);
        naive::State start(0, h);
        checkSensors(0, h, start);
        generateTrajectory(start, tempTraj, lengthIndexNum);
        std::cout << "mincost = " << std::to_string(minCost) << " when start h = " << std::to_string(h) << "\n";
    }
}

void naive::NaiveSolver::checkSensors(int lenId, int heiId, naive::State &state) const {
    state.sensorFlag.resize(sensorNum, false);
    for (int i = 0; i < sensorNum; i++) {
        state.sensorFlag[i] = problem->getSensor(i).isCovered(lenId, heiId);
    }
}

void naive::NaiveSolver::generateTrajectory(naive::State curr, Trajectory &traj, int trajLen) {
    int l = curr.lenId + 1;

    if (l == trajLen) {
        // for (int h : traj.getHeightSche()) {
        //     std::cout << std::to_string(h) << ", ";
        // }
        // std::cout << "\n";
        // 先检查可行性（是否所有传感器的数据都能被收集）
        if (!isFeasible(traj)) {
            std::cout << "Infeasible\n";
            return;
        }
        std::cout << "Feasible\n";
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
    naive::State ss(l, curr.heiId);
    checkSensors(l, curr.heiId, ss);
    candList.push_back(ss);
    for (int h = minHeightIndex; h <= maxHeightIndex; h++) {

        if (h == curr.heiId) continue;

        naive::State s(l, h);
        checkSensors(l, h, s);
        // if (!curr.isSubset(s)) {
        //     candList.push_back(s);
        // } else if (!ss.isSubset(s)) {
        //     candList.push_back(s);
        // }
        if (!ss.isSubset(s)) {
            candList.push_back(s);
        }
    }
    for (naive::State next : candList) {
        traj.setHeightIndex(l, next.heiId);
        generateTrajectory(next, traj, trajLen);
    }
}

bool naive::NaiveSolver::isFeasible(const Trajectory &traj) {
    int count = 0;
    bool vis[sensorNum] = {false};
    for (int d = 0; d < lengthIndexNum && count < sensorNum; d++) {
        int h = traj.getHeightIndex(d);
        for (int s = 0; s < sensorNum; s++) {
            if (vis[s]) continue;
            if (problem->getSensor(s).isCovered(d, h)) {
                vis[s] = true;
                ++count;
            }
        }
    }
    // std::cout << "isFeasible?  count = " << count << " and sensorNum = " << sensorNum << "\n";
    return (count == sensorNum);
}