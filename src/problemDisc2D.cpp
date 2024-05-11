#include "problemDisc2D.h"

#include "acoOnline.h"
#include "onlineACO.h"

ProblemDisc2D::ProblemDisc2D(const Problem2D &prob) {
    unitHeight = resource::REF_UNIT_HEIGHT;
    unitLength = resource::REF_UNIT_LENGTH;

    heightDiscNum = prob.getHeightDiscNum();
    minHeightIndex = 0;
    maxHeightIndex = heightDiscNum - 1;
    minHeight = prob.getMinHeight();

    length = prob.getLength();
    // lengthDiscNum = resource::lengthToIndex(length, 0, unitLength) + 1;
    lengthDiscNum = resource::lengthToIndex(length, 0, unitLength); // ! +1 +1 +1 +1 +1 +1 +1 +1 +1 +1 

    sensorNum = prob.getSensorNum();
    sensorList.resize(sensorNum);
    std::vector<resource::Sensor2D> origin = prob.getSensorList();
    for (int i = 0; i < sensorNum; i++) {
        sensorList[i].time = origin[i].time;
        int temp = origin[i].rangeList.size();
        sensorList[i].rangeList.resize(temp);
        // 离散化
        for (int j = 0; j < temp; j++) {
            sensorList[i].rangeList[j].leftIndex = resource::lengthToIndex(origin[i].rangeList[j].left, 0, unitLength);
            if (resource::indexToLength(sensorList[i].rangeList[j].leftIndex, 0, unitLength) < origin[i].rangeList[j].left) {
                ++sensorList[i].rangeList[j].leftIndex;
            }
            sensorList[i].rangeList[j].rightIndex = resource::lengthToIndex(origin[i].rangeList[j].right, 0, unitLength);
        }
        sensorList[i].setRmost();
    }
}

ProblemDisc2D::ProblemDisc2D(int start, int end, const ProblemOnlineDisc2D &prob, const online::ACOSolver_Online &onlineSolver) {
    unitLength = prob.getUnitLength();
    unitHeight = prob.getUnitHeight();

    heightDiscNum = prob.getHeightDiscNum();
    minHeightIndex = 0;
    maxHeightIndex = heightDiscNum - 1;
    minHeight = prob.getMinHeight();

    // length = prob.getLength();
    // lengthDiscNum = resource::lengthToIndex(length, 0, unitLength) + 1;
    // lengthDiscNum = end - start + 1;
    lengthDiscNum = end - start; // ! +1 +1 +1 +1 +1 +1 +1 +1 +1 +1 
    length = resource::indexToLength(lengthDiscNum, 0, unitLength);

    sensorNum = 0; // 逐个统计active的传感器数量
    sensorIndexMap.clear();
    std::vector<resource::SensorOnlineDisc2D> origin = prob.getSensorList();
    std::vector<online::Sensor> states = onlineSolver.getSensorState();
    for (int i = 0; i < prob.getSensorNum(); i++) {
        // 只采集活跃（已发现，且未传输完成）的传感器
        if (!states[i].isActive()) continue;

        resource::SensorDisc2D sensor;
        sensor.time = states[i].getTime();
        // ! online转为offline时，data range覆盖的高度的数量可能会变小
        sensor.rangeList.clear();
        int temp = origin[i].dataList.size();
        for (int j = 0; j < temp; j++) {
            resource::RangeDisc rg;
            // rg.rightIndex = origin[i].dataList[j].rightIndex - start;
            // if (rg.rightIndex <= 0) break;
            rg.rightIndex = std::max(0, origin[i].dataList[j].rightIndex - start);
            rg.leftIndex = std::max(0, origin[i].dataList[j].leftIndex - start);
            sensor.rangeList.push_back(rg);
        }
        // if (sensor.rangeList.empty()) {
        //     continue;
        // }
        sensor.setRmost();
        // if (sensor.rmost == 0) {
        //     std::cout << "rmost = 0";
        //     std::cout << "\n";
        // }

        sensorIndexMap.push_back(i);
        sensorList.push_back(sensor);
        ++sensorNum;
    }
}

int ProblemDisc2D::getSensorNum() const {
    return sensorNum;
}

double ProblemDisc2D::getLength() const {
    return length;
}

std::vector<resource::SensorDisc2D> ProblemDisc2D::getSensorList() const {
    return sensorList;
}

resource::SensorDisc2D ProblemDisc2D::getSensor(int index) const {
    return sensorList[index];
}

int ProblemDisc2D::getMinHeightIndex() const {
    return minHeightIndex;
}

int ProblemDisc2D::getMaxHeightIndex() const {
    return maxHeightIndex;
}

int ProblemDisc2D::getLengthDiscNum() const {
    return lengthDiscNum;
}

int ProblemDisc2D::getHeightDiscNum() const {
    return heightDiscNum;
}

double ProblemDisc2D::getMinHeight() const {
    return minHeight;
}

int ProblemDisc2D::mapSensor(int offlineIndex) const {
    return sensorIndexMap[offlineIndex];
}

ProblemDisc2D::ProblemDisc2D(int start, int end, const ProblemOnlineDisc2D &prob, const online_aco::OnlineACOSolver &onlineSovler) {
    unitLength = prob.getUnitLength();
    unitHeight = prob.getUnitHeight();
    minHeight = prob.getMinHeight();
    heightDiscNum = prob.getHeightDiscNum();
    minHeightIndex = 0;
    maxHeightIndex = heightDiscNum - 1;
    lengthDiscNum = end - start; // +1
    length = resource::indexToLength(lengthDiscNum, 0, unitLength);

    // sensorNum = 0;
    sensorIndexMap.clear();
    std::vector<resource::SensorOnlineDisc2D> origin = prob.getSensorList();
    std::vector<online_aco::SensorState> states = onlineSovler.getSensorStates();
    int originNum = prob.getSensorNum();
    for (int oid = 0; oid < originNum; oid++) {
        if (!states[oid].isActive()) continue;
        resource::SensorDisc2D sensor;
        sensor.time = states[oid].getTime();
        sensor.rangeList.clear();
        int temp = origin[oid].dataList.size();
        for (int h = 0; h < temp; h++) {
            resource::RangeDisc rg;
            rg.leftIndex = origin[oid].dataList[h].leftIndex;
            rg.rightIndex = origin[oid].dataList[h].rightIndex;
            if (rg.leftIndex < 0) rg.leftIndex = 0;
            if (rg.rightIndex < 0) rg.rightIndex = 0;
            sensor.rangeList.push_back(rg);
        }
        sensor.setRmost();
        sensorIndexMap.push_back(oid);
    }
    sensorNum = sensorIndexMap.size();
}