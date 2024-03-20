#ifndef PROBLEM_2D_H
#define PROBLEM_2D_H

#include <string>
#include <iostream>
#include <fstream>

#include "resource.h"
#include "tools.h"

class Problem2D {
private:
    // 传感器数量
    int sensorNum;
    // 传感器信息表
    vector<resource::Sensor2D> sensorList;
    // 路径总长度
    double length;
    // 飞行高度
    int heightDiscNum;
    vector<double> heightList;
    // 最低飞行高度
    double minHeight;
    // 最高飞行高度
    double maxHeight;
    // 随机数种子
    unsigned int seed;

public:
    int getSensorNum() const;
    double getLength() const;
    vector<resource::Sensor2D> getSensorList() const;
    resource::Sensor2D getSensor(int index) const;
    double getMinHeight() const;
    double getMaxHeight() const;
    int getHeightDiscNum() const;
    vector<double> getHeightList() const;
    void initFromFile(const string &filename);
};

#endif