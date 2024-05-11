#include "onlineACO.h"

#include "problemDisc2D.h"

/* ------------------------------- SensorState ------------------------------ */

double online_aco::SensorState::getTime() const {
    return time;
}

bool online_aco::SensorState::isActive() const {
    return active;
}

void online_aco::SensorState::reduceTime(double dTime) {
    time -= dTime;
}

void online_aco::SensorState::setActive() {
    active = true;
}

void online_aco::SensorState::setInactive() {
    active = false;
}

void online_aco::SensorState::clearTime() {
    time = 0;
}


/* ----------------------------- OnlineACOSolver ---------------------------- */

online_aco::OnlineACOSolver::OnlineACOSolver(ProblemOnlineDisc2D *prob): problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();

    sensorStates.clear();
    for (int i = 0; i < sensorNum; i++) {
        sensorStates.emplace_back(problem->getSensor(i).time);
    }

    cost = hcost = vcost = 0;
}

double online_aco::OnlineACOSolver::getCost() const {
    return cost;
}

double online_aco::OnlineACOSolver::getHcost() const {
    return hcost;
}

double online_aco::OnlineACOSolver::getVcost() const {
    return vcost;
}

std::vector<online_aco::SensorState> online_aco::OnlineACOSolver::getSensorStates() const {
    return sensorStates;
}

void online_aco::OnlineACOSolver::collectData(const std::vector<int> &linked, double v) {
    if (linked.empty()) return;

    // 采集数据
    int num = linked.size();
    double time = problem->getUnitLength() / v;
    for (int i = 0, sid; i < num; i++) {
        sid = linked[i];
        if (!sensorStates[sid].isActive()) continue; // 不活跃传感器，已传输完成，则跳过
        if (sensorStates[sid].getTime() - time > 0) {
            sensorStates[sid].reduceTime(time);
            break;
        }
        // else: 可以把传感器sid的数据采集完
        time -= sensorStates[sid].getTime();
        sensorStates[sid].clearTime();
        sensorStates[sid].setInactive();
    }
}

void online_aco::OnlineACOSolver::updateEnergy(double v, int currh, int nexth) {
    // udpate hcost
    if (currh != nexth) {
        double dh = std::abs(nexth - currh) * problem->getUnitHeight();
        hcost += resource::costByHeight(dh, 1);
    }
    // update vcost
    vcost += resource::costByFly(problem->getUnitLength(), v);
}

std::vector<int> online_aco::OnlineACOSolver::exploreNewSensor(int currd, int currh, const std::vector<resource::SensorOnlineDisc2D> &sensorList, std::vector<bool> &informed, int &currEnd) {
    std::vector<int> newSensors;
    for (int s = 0; s < sensorNum; s++) {
        if (informed[s]) continue;
        if (sensorList[s].controlList[currh].leftIndex <= currd) {
            informed[s] = true;
            newSensors.push_back(s);
            sensorStates[s].setActive();
            // 更新currEnd
            for (resource::RangeDisc rg : sensorList[s].dataList) {
                currEnd = std::max(currEnd, rg.rightIndex);
            }
        }
    }
    return newSensors;
}

void online_aco::OnlineACOSolver::resolve(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked) {
    ProblemDisc2D offlineProb = ProblemDisc2D(start, end, *problem, *this);
    aco::ACOSolver offlineSolver = aco::ACOSolver(&offlineProb);
    // 求解，并获得速度调度和传感器连接方案
    offlineSolver.solveForOnline(start, end, speedSche, linked);
    // 更新路径
    Trajectory traj = offlineSolver.getTrajectory();
    for (int d = start; d < end; d++) {
        trajectory.setHeightIndex(d, traj.getHeightIndex(d - start));
    }
}

void online_aco::OnlineACOSolver::solve(std::vector<double> &speedSche) {
    int countInformed = 0; // 被发现的传感器个数
    std::vector<bool> informed(sensorNum, false); // 对应online中的原传感器编号

    int hMin = problem->getMinHeightIndex();
    int hMax = problem->getMaxHeightIndex();
    std::vector<resource::SensorOnlineDisc2D> sensorList = problem->getSensorList();
    
    int curr = hMin; // 当前高度和下一步高度
    int trajLen = lengthIndexNum; // 路径长度

    // 未获得任何信息时，首先以hmin飞行
    trajectory = Trajectory(trajLen, hMin);

    // 速度调度
    speedSche.clear();
    speedSche.resize(trajLen, resource::V_STAR);

    std::vector<std::vector<int>> linked(trajLen); // 可连接的传感器

    bool newInfo = false; // 是否发现新传感器
    int currEnd = 0; // 当前已获得的信息中data range所能覆盖的最远距离 (index)
    
    // 初始便处于control range里的传感器
    for (int h = hMin; h <= hMax; h++) {
        // newSensors 存的是传感器编号
        std::vector<int> newSensors = exploreNewSensor(0, h, sensorList, informed, currEnd);
        if (!newSensors.empty()) {
            // 发现了新传感器
            newInfo = true;
            countInformed += newSensors.size();
        }
    }

    // for span: (d, d+1)
    for (int d = 0, next; ; ) {
        // 到达终点
        if (d == trajLen) break;
        
        // currEnd = std::max(currEnd, d);
        if (newInfo) {
            // todo complete
            resolve(d, currEnd, speedSche, linked);
        }

        next = trajectory.getHeightIndex(d);
        double v = speedSche[d];

        collectData(linked[d], v);
        updateEnergy(v, curr, next);
        curr = next;
        ++d;

        // 到达新位置后，是否能发现新传感器
        newInfo = false;
        if (countInformed < sensorNum) {
            std::vector<int> newSensors = exploreNewSensor(d, next, sensorList, informed, currEnd);
            if (!newSensors.empty()) {
                // 发现了新传感器
                newInfo = true;
                countInformed += newSensors.size();
            }
        }
    }

    cost = hcost + vcost;
}