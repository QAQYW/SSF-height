#include "resource.h"

/* -------------------------------- parameter ------------------------------- */

const double resource::V_STAR = 13.98; // 在论文里有标注

const double resource::LENGTH_ULP = 0.1;    // ! 慎重取值
const double resource::HEIGHT_ULP = 0.01;   // ! 慎重取值
const double resource::TIME_ULP = 0.1;      // ! 慎重取值
const double resource::ANS_TIME_ULP = resource::TIME_ULP / 100;

const double resource::REF_UNIT_HEIGHT = 10;
const double resource::REF_UNIT_LENGTH = 1; //resource::LENGTH_ULP * 10;

const double resource::HEIGHT_COST_PROPOR = 0.1; //39.095; //10;

/* ----------------------------- data structure ----------------------------- */

void resource::SensorDisc::updateByCoverList() {
    countCover = coverList.size();

    if (coverList.empty()) return;
    
    range.leftIndex = coverList[0]; // ! 段错误
    range.rightIndex = coverList.back();
}

bool resource::SensorDisc2D::isCovered(int lengthIndex, int heightIndex) const {
    if (heightIndex >= rangeList.size()) return false;
    return lengthIndex >= rangeList[heightIndex].leftIndex && lengthIndex <= rangeList[heightIndex].rightIndex;
}

bool resource::SensorOnlineDisc2D::isCovered(int lengthIndex, int heightIndex) const {
    if (heightIndex >= dataList.size()) return false;
    return lengthIndex >= dataList[heightIndex].leftIndex && lengthIndex <= dataList[heightIndex].rightIndex;
}

double resource::power(double v) {
    if (v < 0) {
        std::cout <<"ERROR: Invalid velocity.\n";
        // throw "Invalid velocity.";
    }
    return 0.07 * v * v * v + 0.0391 * v * v - 13.196 * v + 390.95;
}

double resource::costByHeight(double dh) {
    return resource::HEIGHT_COST_PROPOR * dh;
}

double resource::costByHeight(double h1, double h2) {
    return resource::costByHeight(std::abs(h1 - h2));
}

double resource::costByFly(double dis, double v) {
    return dis / v * power(v);
}

int resource::valueToIndex(double val, double minVal, int minId, double unitVal) {
    return (int) ((val - minVal) / unitVal) + minId;
}

int resource::heightToIndex(double hei, double minHei, double unitHei) {
    return resource::valueToIndex(hei, minHei, 0, unitHei);
}

int resource::lengthToIndex(double len, double minLen, double unitLen) {
    return resource::valueToIndex(len, minLen, 0, unitLen);
}

double resource::indexToValue(int id, int minId, double minVal, double unitVal) {
    return (id - minId) * unitVal + minVal;
}

double resource::indexToHeight(int heiId, double minHei, double unitHei) {
    return resource::indexToValue(heiId, 0, minHei, unitHei);
}

double resource::indexToLength(int lenId, double minLen, double unitLen) {
    return resource::indexToValue(lenId, 0, minLen, unitLen);
}

// 计算两个距离差
double resource::indexToDistance(int activeDis, double unitLen) {
    return activeDis * unitLen;
}

// 计算两个lengthIndex之间的距离差（包含两端点）
// disId2 >= disId1
double resource::indexToDistance(int disId1, int disId2, double unitLen) {
    // return indexToDistance(disId2 - disId1, unitLen);
    return indexToDistance(disId2 - disId1 + 1, unitLen);
}

