#include "acoOnline.h"

/* --------------------------------- Sensor --------------------------------- */

double online::Sensor::getTime() const {
    return time;
}

bool online::Sensor::isActive() const {
    return active;
}

void online::Sensor::reduceTime(double dTime) {
    time -= dTime;
}

void online::Sensor::clearTime() {
    time = 0;
}

void online::Sensor::setActive() {
    active = true;
}

void online::Sensor::setInactive() {
    active = false;
}

/* ---------------------------- ACOSolver_Online ---------------------------- */

online::ACOSolver_Online::ACOSolver_Online(ProblemOnlineDisc2D *prob): problem(prob), trajectory() {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();

    rBound.resize(lengthIndexNum, 0);

    sensorState.clear();
    for (int i = 0; i < sensorNum; i++) {
        // 尽管所需传输时间可以先记录下来
        // 但要等进入control range获取信息后才允许访问
        sensorState.emplace_back(online::Sensor(problem->getSensor(i).time));
    }

    cost = hcost = vcost = 0;
}

ProblemOnlineDisc2D* online::ACOSolver_Online::getProblem() const {
    return problem;
}

Trajectory online::ACOSolver_Online::getTrajectory() const {
    return trajectory;
}

std::vector<online::Sensor> online::ACOSolver_Online::getSensorState() const {
    return sensorState;
}

double online::ACOSolver_Online::getCost() const {
    return cost;
}

double online::ACOSolver_Online::getHcost() const {
    return hcost;
}

double online::ACOSolver_Online::getVcost() const {
    return vcost;
}

void online::ACOSolver_Online::solve(std::vector<double> &speedSche) {
    int countInformed = 0;
    std::vector<bool> informed(sensorNum, false); // 对应的是原传感器编号

    int hMin = problem->getMinHeightIndex();
    int hMax = problem->getMaxHeightIndex();
    std::vector<resource::SensorOnlineDisc2D> sensorList = problem->getSensorList();
    
    // 是否获得新的传感器信息
    bool newInfo = false;//true;
    // 当前已获得的信息中range所能覆盖的最远距离 (index)
    int currEnd = 0;
    // 刚开始可以获得所有control range覆盖了d=0的传感器信息
    // 满足 min{sensorList[i].controlList[j] == 0} 即可
    for (int h = hMin; h <= hMax; h++) {
        std::vector<int> newSensors = exploreNewSensor(0, h, sensorList, informed, currEnd);
        if (!newSensors.empty()) {
            newInfo = true;
            countInformed += newSensors.size();
        }
    }

    int curr = hMin, next;
    int trajLen = lengthIndexNum;
    
    // 先默认全程以hMin高度飞行
    trajectory = Trajectory(trajLen, hMin);

    // 速度调度
    speedSche.clear();
    speedSche.resize(trajLen, resource::V_STAR);
    
    // 每个离散位置所连接的传感器
    std::vector<std::vector<int>> linked(trajLen);

    for (int d = 0; /*d < trajLen*/; /*d++*/) {
        
        if (d == trajLen) break; // ?????????

        // 若获得了新的传感器信息，则需重新规划
        if (newInfo) {
            resolve(d, currEnd, speedSche, linked);
        }
        
        next = trajectory.getHeightIndex(d);
        double v = speedSche[d];

        // 采集数据
        collectData(linked[d], v);
        // 统计无人机能耗（hcost & vcost）
        updateEnergy(v, curr, next);
        // 更新无人机的高度
        curr = next;

        ++d;

        // 尝试获得新的传感器信息（若有）
        newInfo = false;
        if (countInformed < sensorNum) {
            std::vector<int> newSensors = exploreNewSensor(d, next, sensorList, informed, currEnd);
            if (!newSensors.empty()) {
                newInfo = true;
                countInformed += newSensors.size();
            }
        }
    }
    // 统计 cost
    cost = hcost + vcost;
}

void online::ACOSolver_Online::resolve(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked) {
    // std::cout << "in func: online::ACOSolver_Online::resolve(...)\n";
    ProblemDisc2D offlineProb = ProblemDisc2D(start, end, *problem, *this);
    // std::cout << "sensorNum = " << std::to_string(problem->getSensorNum()) << "\n";
    // std::cout << "sensorNum = " << std::to_string(offlineProb.getSensorNum()) << "\n";
    aco::ACOSolver offlineSolver(&offlineProb);
    // 求解，并获得速度调度和传感器连接方案
    offlineSolver.solveForOnline(start, end, speedSche, linked);
    // 更新 trajectory
    Trajectory subOptTraj = offlineSolver.getTrajectory();
    for (int i = start; i < end; i++) {
        trajectory.setHeightIndex(i, subOptTraj.getHeightIndex(i - start));
    }
    // std::cout << "check trajectory\n";
}

void online::ACOSolver_Online::collectData(const std::vector<int> &linked, double v) {
    if (linked.empty()) {
        // std::cout << "empty linked in function 'collectData'\n";
        return;
    }
    // 采集数据
    int num = linked.size();
    double time = problem->getUnitLength() / v;
    for (int i = 0; i < num; i++) {
        // std::cout << "collect data " << std::to_string(linked[i]) << "\n";
        if (!sensorState[linked[i]].isActive()) continue;
        if (tools::approx(sensorState[linked[i]].getTime() - time, resource::ANS_TIME_ULP) > 0) {
            sensorState[linked[i]].reduceTime(time);
            break;
        }
        // if (sensorState[linked[i]].getTime() >= time) {
        //     sensorState[linked[i]].reduceTime(time);
        //     break;
        // }
        // 若有传感器的数据采集完成，则设为不活跃
        time -= sensorState[linked[i]].getTime();
        sensorState[linked[i]].clearTime();
        sensorState[linked[i]].setInactive();
    }
}

// ! 弃用
void online::ACOSolver_Online::collectData(const std::vector<int> &linked, double v, int &countActive) {
    if (linked.empty()) {
        // std::cout << "empty linked in function 'collectData'\n";
        return;
    }
    // 采集数据
    int num = linked.size();
    double time = problem->getUnitLength() / v;
    for (int i = 0; i < num; i++) {
        if (sensorState[linked[i]].getTime() >= time) {
            sensorState[linked[i]].reduceTime(time);
            break;
        }
        // 若有传感器的数据采集完成，则设为不活跃
        time -= sensorState[linked[i]].getTime();
        sensorState[linked[i]].clearTime();
        sensorState[linked[i]].setInactive();
        --countActive;
    }
}

void online::ACOSolver_Online::updateEnergy(double v, int currh, int nexth) {
    if (currh != nexth) {
        // 高度差的绝对值（真实值），所以要乘上unitHeight
        double dh = std::abs(nexth - currh) * problem->getUnitHeight();
        hcost += resource::costByHeight(dh, 1);
    }
    vcost += resource::costByFly(problem->getUnitLength(), v);
}

std::vector<int> online::ACOSolver_Online::exploreNewSensor(int currd, int currh, const std::vector<resource::SensorOnlineDisc2D> &sensorList, std::vector<bool> &informed, int &currEnd) {
    std::vector<int> newSensors;
    for (int s = 0; s < sensorNum; s++) {
        if (informed[s]) continue;
        if (sensorList[s].controlList[currh].leftIndex <= currd) {
            informed[s] = true;
            newSensors.push_back(s);
            sensorState[s].setActive();
            // 初始传输时间在初始化时已经设置过
            // sensorState[s].setTime(sensorList[s].time);
            for (resource::RangeDisc rg : sensorList[s].dataList) {
                currEnd = std::max(currEnd, rg.rightIndex);
            }
        }
    }
    return newSensors;
}