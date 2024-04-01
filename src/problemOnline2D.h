#ifndef PROBLEM_ONLINE_2D_H
#define PROBLEM_ONLINE_2D_H

#include <string>
#include <iostream>
#include <fstream>

#include "resource.h"
#include "tools.h"

class ProblemOnline2D {

private:
    std::vector<resource::SensorOnline2D> sensorList; // 传感器集合
    int sensorNum; // 传感器数量
    double length; // 路径总长度
    int heightDiscNum;  // 离散的高度数量
    double minHeight;   // 最低飞行高度
    double maxHeight;   // 最高飞行高度
    std::vector<double> heightList;  // 飞行高度
    unsigned int seed;  // 随机数种子

public:
    int getSensorNum() const;
    double getLength() const;
    std::vector<resource::SensorOnline2D> getSensorList() const;
    resource::SensorOnline2D getSensor(int index) const;
    double getMinHeight() const;
    double getMaxHeight() const;
    int getHeightDiscNum() const;
    // 读取文件，初始化
    void initFromFile(const std::string &filename);
};

#endif