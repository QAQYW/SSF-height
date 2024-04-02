#include "problemOnlineDisc2D.h"

ProblemOnlineDisc2D::ProblemOnlineDisc2D(const ProblemOnline2D &prob) {
    unitHeight = resource::REF_UNIT_HEIGHT;
    unitLength = resource::REF_UNIT_LENGTH;

    heightDiscNum = prob.getHeightDiscNum();
    minHeightIndex = 0;
    maxHeightIndex = heightDiscNum - 1;
    minHeight = prob.getMinHeight();

    length = prob.getLength();
    lengthDiscNum = resource::lengthToIndex(length, 0, unitLength) + 1;

    // std::cout << "\nbasic assignment\n";

    sensorList.clear();
    // std::cout << "\nclear 'sensorList'\n";

    sensorNum = prob.getSensorNum();
    std::cout << "\nsensorNum = " << std::to_string(sensorNum) << "\n";
    sensorList.resize(sensorNum);
    // std::cout << "\nresize 'sensorList'\n";

    std::vector<resource::SensorOnline2D> origin = prob.getSensorList();
    // std::cout << "\ncreate new vector 'origin'\n";
    
    std::cout << "\norigin.size() = " << std::to_string(origin.size()) << "\n";

    for (int i = 0; i < sensorNum; i++) {
        sensorList[i].time = origin[i].time;
        int temp = origin[i].dataList.size();
        sensorList[i].dataList.resize(temp);
        sensorList[i].controlList.resize(temp);
        for (int j = 0; j < temp; j++) {
            sensorList[i].dataList[j].leftIndex     = resource::lengthToIndex(origin[i].dataList[j].left, 0, unitLength);
            sensorList[i].dataList[j].rightIndex    = resource::lengthToIndex(origin[i].dataList[j].right, 0, unitLength);
            sensorList[i].controlList[j].leftIndex  = resource::lengthToIndex(origin[i].controlList[j].left, 0, unitLength);
            sensorList[i].controlList[j].rightIndex = resource::lengthToIndex(origin[i].controlList[j].right, 0, unitLength);
        }
    }

    // 手动释放
    // delete &origin;
    // origin.~vector();
}

int ProblemOnlineDisc2D::getSensorNum() const {
    return sensorNum;
}

double ProblemOnlineDisc2D::getLength() const {
    return length;
}

std::vector<resource::SensorOnlineDisc2D> ProblemOnlineDisc2D::getSensorList() const {
    return sensorList;
}

resource::SensorOnlineDisc2D ProblemOnlineDisc2D::getSensor(int index) const {
    return sensorList[index];
}

int ProblemOnlineDisc2D::getMinHeightIndex() const {
    return minHeightIndex;
}

int ProblemOnlineDisc2D::getMaxHeightIndex() const {
    return maxHeightIndex;
}

int ProblemOnlineDisc2D::getLengthDiscNum() const {
    return lengthDiscNum;
}

int ProblemOnlineDisc2D::getHeightDiscNum() const {
    return heightDiscNum;
}

double ProblemOnlineDisc2D::getMinHeight() const {
    return minHeight;
}

double ProblemOnlineDisc2D::getUnitLength() const {
    return unitLength;
}

double ProblemOnlineDisc2D::getUnitHeight() const {
    return unitHeight;
}