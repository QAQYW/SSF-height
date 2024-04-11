#include "problemDisc2D.h"

#include "acoOnline.h"

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
    std::vector<resource::SensorOnlineDisc2D> origin = prob.getSensorList();
    std::vector<online::Sensor> states = onlineSolver.getSensorState();
    for (int i = 0; i < prob.getSensorNum(); i++) {
        // 只采集活跃（已发现，且未传输完成）的传感器
        if (!states[i].isActive()) continue;

        sensorIndexMap.push_back(i);

        resource::SensorDisc2D sensor;
        sensor.time = states[i].getTime();
        int temp = origin[i].dataList.size();
        sensor.rangeList.resize(temp);
        for (int j = 0; j < temp; j++) {
            // 要减去start
            sensor.rangeList[j].leftIndex = std::max(0, origin[i].dataList[j].leftIndex - start);
            // sensor.rangeList[j].rightIndex = origin[i].dataList[j].rightIndex - start;
            sensor.rangeList[j].rightIndex = std::max(0, origin[i].dataList[j].rightIndex - start);
            // !!!!!!!!!! 是否会有rightIndex小于0的情况
            // if (sensor.rangeList[j].rightIndex < 0) {
            //     std::cout << "ERROR: negative right index\n";
            // }
        }
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

// ProblemDisc1D ProblemDisc2D::transformToProblemDisc1D(const Trajectory &traj) const {
//     ProblemDisc1D prob1D = ProblemDisc1D(sensorNum, length, lengthDiscNum);
// }