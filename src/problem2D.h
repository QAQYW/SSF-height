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
    std::vector<resource::Sensor2D> sensorList;
    // 路径总长度
    double length;
    // 离散的高度数量
    int heightDiscNum;
    std::vector<double> heightList;
    // 最低飞行高度
    double minHeight;
    // 最高飞行高度
    double maxHeight;
    // 随机数种子
    unsigned int seed;

public:
    int getSensorNum() const;
    double getLength() const;
    std::vector<resource::Sensor2D> getSensorList() const;
    resource::Sensor2D getSensor(int index) const;
    double getMinHeight() const;
    double getMaxHeight() const;
    int getHeightDiscNum() const;
    std::vector<double> getHeightList() const;
    /// @brief 从离线问题的文件读取信息
    /// @param filename 文件名
    void initFromFile(const std::string &filename);
    /// @brief 从在线问题的文件读取信息（忽略control range）
    /// @param filename 文件名
    void initFromOnlineFile(const std::string &filename);
};

#endif