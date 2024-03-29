#include "problemDisc2D.h"
using namespace resource;

/* ------------------------------ ProblemDisc2D ----------------------------- */

ProblemDisc2D::ProblemDisc2D(const Problem2D &prob) {
    unitHeight = resource::REF_UNIT_HEIGHT;
    unitLength = resource::REF_UNIT_LENGTH;

    heightDiscNum = prob.getHeightDiscNum();
    minHeightIndex = 0;
    maxHeightIndex = heightDiscNum - 1;
    minHeight = prob.getMinHeight();

    length = prob.getLength();
    lengthDiscNum = resource::lengthToIndex(length, 0, unitLength) + 1;

    sensorNum = prob.getSensorNum();
    sensorList.resize(sensorNum);
    vector<resource::Sensor2D> origin = prob.getSensorList();
    for (int i = 0; i < sensorNum; i++) {
        sensorList[i].time = origin[i].time;
        int temp = origin[i].rangeList.size();
        sensorList[i].rangeList.resize(temp);
        for (int j = 0; j < temp; j++) {
            sensorList[i].rangeList[j].leftIndex = resource::lengthToIndex(origin[i].rangeList[j].left, 0, unitLength);
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
    lengthDiscNum = end - start + 1;
    length = resource::indexToLength(end - start, 0, unitLength);

    // sensorNum = prob.getSensorNum();
    // sensorList.resize(sensorNum);
    sensorNum = 0;
    vector<resource::SensorOnlineDisc2D> origin = prob.getSensorList();
    vector<online::Sensor> states = onlineSolver.getSensorState();
    for (int i = 0; i < sensorNum; i++) {

        if (!states[i].isActive()) continue;

        ++sensorNum;
        sensorIndexMap.push_back(i);

        resource::SensorDisc2D sensor;
        sensor.time = states[i].getTime(); // 应该是online problem中的剩余时间
        int temp = origin[i].dataList.size();
        sensor.rangeList.resize(temp);
        for (int j = 0; j < temp; j++) {
            sensor.rangeList[j].leftIndex = origin[i].dataList[j].leftIndex - start;
            sensor.rangeList[j].rightIndex = origin[i].dataList[j].rightIndex - start;
        }
        sensorList.push_back(sensor);
    }
}

int ProblemDisc2D::getSensorNum() const {
    return sensorNum;
}

double ProblemDisc2D::getLength() const {
    return length;
}

vector<SensorDisc2D> ProblemDisc2D::getSensorList() const {
    return sensorList;
}

SensorDisc2D ProblemDisc2D::getSensor(int index) const {
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