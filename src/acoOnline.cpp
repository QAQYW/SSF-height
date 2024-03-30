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
    // TODO 这部分得等进入了control range才能更新

    sensorState.clear();
    for (int i = 0; i < sensorNum; i++) {
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

void online::ACOSolver_Online::solve() {
    int countInformed = 0;
    vector<bool> informed(sensorNum, false); // 对应的是原传感器编号
    vector<bool> visited(sensorNum, false);  // 对应的是原传感器编号

    int hMin = problem->getMinHeightIndex();
    int hMax = problem->getMaxHeightIndex();
    vector<resource::SensorOnlineDisc2D> sensorList = problem->getSensorList();

    // TODO 获取初始传感器信息
    // TODO 初始化 this->sensors，或在构造函数中初始化
    // for (int i = 0; i < sensorNum; i++) {
    //     bool flag = false;
    //     for (int h = hMin; h <= hMax && !flag; h++) {
    //         if (sensorList[i].controlList[h].leftIndex == 0) {
    //             flag = true;
    //         }
    //     }
    // }

    // 刚开始可以获得所有control range覆盖了d=0的info of sensor
    // 满足 min{sensorList[i].controlList[j] == 0} 即可
    int tempEnd = 0;
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
                tempEnd = max(tempEnd, sensorList[s].dataList[h].rightIndex);
            }
        }
    }

    int curr = hMin, next;
    int trajLen = lengthIndexNum;
    
    trajectory = aco::Trajectory(trajLen, hMin); // 先默认全程以hMin高度飞行
    
    bool newInfo = true;// 是否获得新的传感器信息

    // 速度调度，-1 表示尚未规划
    vector<double> speedSche(trajLen, -1);
    // 每个离散位置所连接的传感器
    vector<vector<int>> linked(trajLen);

    for (int d = 0; d < trajLen; d++) {
        // TODO 若获得了新的信息，则重新求解当前问题；否则，按原方案飞行
        double v;
        vector<int> linked;
        if (newInfo) {
            // 重新规划
            resolve(d, tempEnd);
            // TODO choose the height of next position
            // next = ...
        } else {
            // 按原计划
            // TODO choose the height of next position
            next = trajectory.getHeightIndex(d);
            // ? v = ???
            v = speedSche[d];
            // ? linked = ???
            // ...
        }

        // TODO 选择在这一段要连接的传感器（集合）
        // vector<int> linked; // 在下一状态准备连接的传感器（集合）
        // TODO 对 linkedSensor 中的传感器排个顺序

        // TODO 采集数据
        collectData(linked, v);
        updateEnergy(v, curr, next);

        // TODO 更新无人机的高度
        curr = next;
        
        // TODO 获得新的传感器信息（若有）
        newInfo = false;
        if (countInformed < sensorNum) {
            vector<int> newSensors = exploreNewSensor(d, next, sensorList, informed);
            if (!newSensors.empty()) {
                newInfo = true;
                countInformed += newSensors.size();
            }
        }
        // TODO 更新 visited vector
    }
    cost = hcost + vcost;
}

// TODO: complete
void online::ACOSolver_Online::resolve(int start, int end) {
    ProblemDisc2D offlineProb(start, end, *problem, *this);
    aco::ACOSolver offlineSolver(&offlineProb);
    // TODO
    // TODO
    // TODO
    // TODO
    // TODO
}

void online::ACOSolver_Online::collectData(const vector<int> &linked, double v) {
    // TODO 采集数据
    int num = linked.size();
    double time = problem->getUnitLength() / v;
    for (int i = 0; i < num; i++) {
        if (sensorState[linked[i]].getTime() >= time) {
            sensorState[linked[i]].reduceTime(time);
            break;
        }
        time -= sensorState[linked[i]].getTime();
        sensorState[linked[i]].clearTime();
        sensorState[linked[i]].setInactive();
    }
}

void online::ACOSolver_Online::updateEnergy(double v, int currh, int nexth) {
    hcost += resource::costByHeight(currh, nexth);
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