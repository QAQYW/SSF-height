#ifndef PROBLEM_DISC_1D_H
#define PROBLEM_DISC_1D_H

#include "resource.h"
#include "problemDisc2D.h"
#include "aco.h"

class ProblemDisc2D;
namespace aco {
    class Trajectory;
}

class ProblemDisc1D {
private:
    // 传感器数量
    int sensorNum;
    // 传感器列表
    vector<resource::SensorDisc> sensorList;
    // 路径总长度
    double length;
    // 离散化后从x=0到x=length共有多少个单位距离，编号范围0~lengthDiscNum-1
    int lengthDiscNum;

public:
    void transformFromProblemDisc2D(const ProblemDisc2D &prob, const aco::Trajectory &traj);
    vector<resource::SensorDisc> getSensorList() const;
    resource::SensorDisc getSensor(int index) const;
    double getLength() const;
    int getLengthDiscNum() const;
    int getSensorNum() const;
};

#endif