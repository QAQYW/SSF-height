#ifndef PROBLEM_DISC_1D_H
#define PROBLEM_DISC_1D_H

#include "resource.h"
#include "trajectory.h"
// #include "problemDisc2D.h"

// 前置声明
// class Trajectory;
// class ProblemDisc2D;

class ProblemDisc1D {
private:
    // 传感器数量
    int sensorNum;
    // 传感器列表
    std::vector<resource::SensorDisc> sensorList;
    // 路径总长度
    double length;
    // 离散化后从x=0到x=length共有多少个单位距离，编号范围0~lengthDiscNum-1
    int lengthDiscNum;

public:
    /// @brief 构造函数
    /// @param num 传感器数量 sensorNum
    /// @param len 路径长度 length
    /// @param lenNum 路径长度离散后的数量 lengthDiscNum
    /// @param list ProblemDisc2D的传感器集合
    /// @param traj 轨迹
    ProblemDisc1D(int num, double len, int lenNum, const std::vector<resource::SensorDisc2D> &list, const Trajectory &traj);
    std::vector<resource::SensorDisc> getSensorList() const;
    resource::SensorDisc getSensor(int index) const;
    double getLength() const;
    int getLengthDiscNum() const;
    int getSensorNum() const;
};


#endif