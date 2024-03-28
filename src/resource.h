#ifndef RESOURCE_H
#define RESOURCE_H

#include <vector>
#include <cmath>
#include <iostream>
using namespace std;

namespace resource {
    /* -------------------------------- parameter ------------------------------- */
    extern const double V_STAR;         // 最小能耗速度
    // TODO 离散化的单位要重新定义新的，不是ULP
    extern const double LENGTH_ULP;     // ? 是精度 不是距离离散化的单位
    extern const double HEIGHT_ULP;     // ? 是精度 不是高度离散化的单位
    extern const double HEIGHT_COST_PROPOR; // 高度变化造成的能耗的比例
    extern const double TIME_ULP;       // 时间精度

    extern const double REF_UNIT_HEIGHT; // 高度离散化间隔的参考值
    extern const double REF_UNIT_LENGTH; // 距离离散化间隔的参考值

    /* -------------------------- basic data structure -------------------------- */
    // 范围/区间
    struct Range {
        double left;
        double right;
        Range() {};
        Range(double left, double right): left(left), right(right) {};
    };
    // 离散化的范围/区间
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
        vector<int> coverList;  // 可能不连续的，是lengthIndex的vector
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
        vector<Range> rangeList;
    };
    // 距离离散化的传感器 2D
    struct SensorDisc2D {
        double time;
        vector<RangeDisc> rangeList;
        bool isCovered(int lengthIndex, int heightIndex) const;
    };
    // 传感器2D online
    struct SensorOnline2D {
        double time;
        vector<Range> dataList; // data transmission ranges
        vector<Range> controlList; // control communication ranges
    };
    // 距离离散化的传感器2D online
    struct SensorOnlineDisc2D {
        double time;
        vector<RangeDisc> dataList;
        vector<RangeDisc> controlList;
        bool isCovered(int lengthIndex, int heightIndex) const;
    };

    /* -------------------------------- function -------------------------------- */
    // 速度-功率函数
    double power(double v);
    // 高度变化产生的能耗
    double costByHeight(double dh);
    double costByHeight(double h1, double h2);
    // 固定高度飞行一段距离产生的能耗
    double costByFly(double dis, double v);
    // 离散化
    int valueToIndex(double val, double minVal, int minId, double unitVal);
    int heightToIndex(double hei, double minHei, double unitHei);
    int lengthToIndex(double len, double minLen, double unitLen);
    // int lengthToIndex(double len);
    // 恢复真实值
    double indexToValue(int id, int minId, double minVal, double unitVal);
    double indexToHeight(int heiId, double minHei, double unitHei);
    double indexToLength(int lenId, double minLen, double unitLen);
    // 计算距离差
    double indexToDistance(int activeDis, double unitLen);
    double indexToDistance(int disId1, int disId2, double unitLen);
}

#endif