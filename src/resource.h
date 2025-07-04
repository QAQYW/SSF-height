#ifndef RESOURCE_H
#define RESOURCE_H

#include <vector>
#include <cmath>
#include <iostream>

namespace resource {

/* -------------------------------- parameter ------------------------------- */

extern const double V_STAR;         // 最小能耗速度

extern const double LENGTH_ULP;     // ? 是精度 不是距离离散化的单位
extern const double HEIGHT_ULP;     // ? 是精度 不是高度离散化的单位
extern const double TIME_ULP;       // 时间精度
extern const double ANS_TIME_ULP; // 计算答案（剩余传输时间时的时间精度）在online solver中用到

extern double REF_UNIT_HEIGHT; // 高度离散化间隔的参考值
extern const double REF_UNIT_LENGTH; // 距离离散化间隔的参考值

/// @brief 升降单位高度的能耗，量纲：焦耳/米 (J/m)
/// 推导过程见飞书文档
/// https://seunetsi.feishu.cn/docx/PfNvd5m5bohZfUxkJxycEtb4nQe
extern double HEIGHT_COST_PROPOR;
extern const double HEIGHT_COST_BIAS;

/* ----------------------------- data structure ----------------------------- */

// 传输范围
struct Range {
    double left;
    double right;
    Range() {};
    Range(double left, double right): left(left), right(right) {};
};
// 离散化的范围
struct RangeDisc {
    int leftIndex;
    int rightIndex;
    RangeDisc() {};
    RangeDisc(int leftIndex, int rightIndex): leftIndex(leftIndex), rightIndex(rightIndex) {};
};
// 传感器 1D
struct Sensor {
    Range range;
    double time;
    Sensor(Range range, double time): range(range), time(time) {};
};
// 距离离散化的传感器 1D
struct SensorDisc {
    RangeDisc range;        // 最左与最右端点构成的范围
    std::vector<int> coverList;  // 可能不连续的，是lengthIndex的vector
    int countCover;         // coverList.size()
    double length;          // = countCover * unitLength
    double time;
    SensorDisc(): countCover(0) {};
    // void init();
    void updateByCoverList();
};
// 传感器 2D
struct Sensor2D {
    double time;
    std::vector<Range> rangeList;
};
// 距离离散化的传感器 2D
struct SensorDisc2D {
    double time;
    std::vector<RangeDisc> rangeList;
    int rmost;
    bool isCovered(int lengthIndex, int heightIndex) const;
    void setRmost();
};
// 传感器2D online
struct SensorOnline2D {
    double time;
    std::vector<Range> dataList; // data transmission ranges
    std::vector<Range> controlList; // control communication ranges
};
// 距离离散化的传感器2D online
struct SensorOnlineDisc2D {
    double time;
    std::vector<RangeDisc> dataList;
    std::vector<RangeDisc> controlList;
    int rmost;
    bool isCovered(int lengthIndex, int heightIndex) const;
    void setRmost();
};

/* -------------------------------- function -------------------------------- */
// 速度-功率函数
double power(double v);
// 高度变化产生的能耗
double costByHeight(double dh, double coef);
double costByHeight(double h1, double h2, double coef);
// 固定高度飞行一段距离产生的能耗
double costByFly(double dis, double v);
// 离散化
int valueToIndex(double val, double minVal, int minId, double unitVal);
int heightToIndex(double hei, double minHei, double unitHei);
int lengthToIndex(double len, double minLen, double unitLen);
// 恢复真实值
double indexToValue(int id, int minId, double minVal, double unitVal);
double indexToHeight(int heiId, double minHei, double unitHei);
double indexToLength(int lenId, double minLen, double unitLen);
// 计算距离差
double indexToDistance(int activeDis, double unitLen);
double indexToDistance(int disId1, int disId2, double unitLen);


/// @brief 根据速度调度，计算无人机的飞行时间
/// @param uLength 单位长度
/// @param speedSche 速度调度方案
/// @return 飞行时间
double calFlightTime(double uLength, const std::vector<double> speedSche);

}

#endif