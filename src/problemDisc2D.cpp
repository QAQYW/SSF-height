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
    lengthDiscNum = resource::lengthToIndex(length, 0, unitLength);

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