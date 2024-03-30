#ifndef PROBLEM_DISC_2D_H
#define PROBLEM_DISC_2D_H

#include "resource.h"
// #include "problemDisc1D.h"
#include "problem2D.h"
#include "problemOnlineDisc2D.h"
#include "acoOnline.h"

class ProblemDisc2D {
private:
    // 传感器数量
    int sensorNum;
    // 传感器列表
    vector<resource::SensorDisc2D> sensorList;
    // 路径总长度
    double length;
    // 最低飞行高度
    double minHeight;
    // 最低飞行高度编号
    int minHeightIndex;
    // 最高飞行高度编号
    int maxHeightIndex;
    // 离散化后从x=0到x=length共有多少个单位距离，编号范围0~lengthDiscNum-1
    int lengthDiscNum;
    // 离散化后从minHeight到maxHeight共有多少个单位高度，编号范围0~heightDiscNum-1
    int heightDiscNum;
    // 最小长度单位
    double unitLength;
    // 最小高度单位
    double unitHeight;
    // 与 online problem 的传感器编号映射
    vector<int> sensorIndexMap;

public:
    ProblemDisc2D(const Problem2D &prob);
    ProblemDisc2D(int start, int end, const ProblemOnlineDisc2D &prob, const online::ACOSolver_Online &onlineSolver);
    int getSensorNum() const;
    double getLength() const;
    vector<resource::SensorDisc2D> getSensorList() const;
    resource::SensorDisc2D getSensor(int index) const;
    int getMinHeightIndex() const;
    int getMaxHeightIndex() const;
    int getLengthDiscNum() const;
    int getHeightDiscNum() const;
    double getMinHeight() const;
    int mapSensor(int offlineIndex) const;
};

#endif