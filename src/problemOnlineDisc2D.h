#ifndef PROBLEM_ONLINE_DISC_2D_H
#define PROBLEM_ONLINE_DISC_2D_H

#include "resource.h"
#include "problemOnline2D.h"
#include "aco.h"

class ProblemOnlineDisc2D {

// ! control communication range 必须覆盖所有高度
// ! 否则可能无人机全程都无法获得该传感器的信息

private:
    vector<resource::SensorOnlineDisc2D> sensorList; // 传感器列表
    int sensorNum;  // 传感器数量
    double length;  // 路径总长度
    double minHeight;   // 最低飞行高度
    int minHeightIndex; // 最低飞行高度编号
    int maxHeightIndex; // 最高飞行高度编号
    int lengthDiscNum;  // 离散化后从x=0到x=length共有多少个单位距离，编号范围0~lengthDiscNum-1
    int heightDiscNum;  // 离散化后从minHeight到maxHeight共有多少个单位高度，编号范围0~heightDiscNum-1
    double unitLength;  // 距离离散化的间隔
    double unitHeight;  // 高度离散化的间隔

public:
    ProblemOnlineDisc2D(const ProblemOnline2D &prob);
    int getSensorNum() const;
    double getLength() const;
    vector<resource::SensorOnlineDisc2D> getSensorList() const;
    resource::SensorOnlineDisc2D getSensor(int index) const;
    int getMinHeightIndex() const;
    int getMaxHeightIndex() const;
    int getLengthDiscNum() const;
    int getHeightDiscNum() const;
    double getMinHeight() const;
    double getUnitLength() const;
    double getUnitHeight() const;
};


#endif