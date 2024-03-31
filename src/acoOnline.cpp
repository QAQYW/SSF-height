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
        // 所需传输时间可以先记录下来
        // 但要等进入control range获取信息后才允许访问
        sensorState.emplace_back(online::Sensor(problem->getSensor(i).time));
    }

    cost = hcost = vcost = 0;
}

ProblemOnlineDisc2D* online::ACOSolver_Online::getProblem() const {
    return problem;
}

aco::Trajectory online::ACOSolver_Online::getTrajectory() const {
    return trajectory;
}

vector<online::Sensor> online::ACOSolver_Online::getSensorState() const {
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

void online::ACOSolver_Online::solve(vector<double> &speedSche) {
    int countInformed = 0;
    vector<bool> informed(sensorNum, false); // 对应的是原传感器编号
    // vector<bool> visited(sensorNum, false);  // 对应的是原传感器编号

    int hMin = problem->getMinHeightIndex();
    int hMax = problem->getMaxHeightIndex();
    vector<resource::SensorOnlineDisc2D> sensorList = problem->getSensorList();

    // 当前已获得的信息中range所能覆盖的最远距离 (index)
    int currEnd = 0;
    // 刚开始可以获得所有control range覆盖了d=0的传感器信息
    // 满足 min{sensorList[i].controlList[j] == 0} 即可
    for (int s = 0; s < sensorNum; s++) {
        for (int h = hMin; h <= hMax; h++) {
            if (sensorList[s].controlList[h].leftIndex == 0) {
                informed[s] = true;
                ++countInformed;
                break;
            }
        }
        if (informed[s]) {
            sensorState[s].setActive();
            for (int h = hMin; h <= hMax; h++) {
                currEnd = max(currEnd, sensorList[s].dataList[h].rightIndex);
            }
        }
    }
    // 是否获得新的传感器信息
    bool newInfo = true;

    int curr = hMin, next;
    int trajLen = lengthIndexNum;
    
    // 先默认全程以hMin高度飞行
    trajectory = aco::Trajectory(trajLen, hMin);

    // 速度调度，-1 表示尚未规划
    // vector<double> speedSche(trajLen, -1);
    speedSche.clear();
    speedSche.resize(trajLen, -1);
    // 每个离散位置所连接的传感器
    vector<vector<int>> linked(trajLen);

    for (int d = 0; d < trajLen; d++) {
        // 若获得了新的传感器信息，则需重新规划
        if (newInfo) resolve(d, currEnd, speedSche, linked); // ? 还需要记录什么信息
        
        next = trajectory.getHeightIndex(d);
        double v = speedSche[d];

        // 采集数据
        collectData(linked[d], v);
        // 统计无人机能耗（hcost & vcost）
        updateEnergy(v, curr, next);
        // 更新无人机的高度
        curr = next;
        // 尝试获得新的传感器信息（若有）
        newInfo = false;
        if (countInformed < sensorNum) {
            vector<int> newSensors = exploreNewSensor(d, next, sensorList, informed);
            if (!newSensors.empty()) {
                newInfo = true;
                countInformed += newSensors.size();
            }
        }
        // // TODO 更新 visited vector
    }
    // 统计 cost
    cost = hcost + vcost;
}

// TODO: complete
void online::ACOSolver_Online::resolve(int start, int end, vector<double> &speedSche, vector<vector<int>> &linked) {
    ProblemDisc2D offlineProb(start, end, *problem, *this);
    aco::ACOSolver offlineSolver(&offlineProb);
    // 求解，并获得速度调度和传感器连接方案
    offlineSolver.solveForOnline(start, end, speedSche, linked);
    // 更新 trajectory
    aco::Trajectory subOptTraj = offlineSolver.getTrajectory();
    for (int i = start; i <= end; i++) {
        trajectory.setHeightIndex(i, subOptTraj.getHeightIndex(i - start));
    }
}

void online::ACOSolver_Online::collectData(const vector<int> &linked, double v) {
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
    }
}

void online::ACOSolver_Online::updateEnergy(double v, int currh, int nexth) {
    if (currh != nexth) {
        double dh = std::abs(nexth - currh) * problem->getUnitHeight();
        // hcost += resource::costByHeight(currh, nexth); // ! 这里错了，参数应该是实际高度，不是索引
        hcost += resource::costByHeight(dh);
    }
    vcost += resource::costByFly(problem->getUnitLength(), v);
}

vector<int> online::ACOSolver_Online::exploreNewSensor(int currd, int currh, const vector<resource::SensorOnlineDisc2D> &sensorList, vector<bool> &informed) {
    vector<int> newSensors;
    for (int s = 0; s < sensorNum; s++) {
        if (informed[s]) continue;
        if (sensorList[s].controlList[currh].leftIndex <= currd) {
            informed[s] = true;
            newSensors.push_back(s);
            sensorState[s].setActive();
            // sensorState[s].setTime(sensorList[s].time);
        }
    }
}