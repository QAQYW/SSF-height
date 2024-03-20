#ifndef SSF_H
#define SSF_H

#include <algorithm>

#include "resource.h"
#include "problemDisc1D.h"

// 前置声明
class ProblemDisc1D;
class ProblemDisc2D;

namespace ssf {

// 解：速度调度
class Solution {
private:
    vector<double> speedSche;

public:
    // 构造函数，速度默认初始化为 resource::V_STAR
    Solution(int size);
    // 构造函数，速度初始化为 v
    Solution(int size, double v);
    // 改变某位置的速度
    void changeSpeedSche(int index, int v);
    // 获得速度调度
    vector<double> getSpeedSche() const;
    // 计算速度调度的能耗
    double calCost() const;
};

class Sensor {
private:
    bool active;        // 是否活跃（未被收集）
    int sensorIndex;    // 传感器的原编号
    int leftIndex;      // 范围左端点的位置
    int rightIndex;     // 范围右端点的位置

public:
    Sensor(int i, int l, int r): active(true), sensorIndex(i), leftIndex(l), rightIndex(r) {};
    // 等价于 getActive()
    bool isActive() const;
    // set active as true
    void setActive();
    // set active as false
    void setInactive();
    int getSensorIndex() const;
    int getLeftIndex() const;
    int getRightIndex() const;
    // overload "<"
    bool operator< (const Sensor& _sensor) const;
};

class Segment {
private:
    int left;
    int right;
    int activeDistance;
    double activeTime;
    double velocity;
    vector<int> sensorList;

public:
    Segment(int l, int r, int d, double t);
    double getVelocity() const;
    int getLeft() const;
    int getRight() const;
    vector<int> getSensorList() const;
    // 增加新发现的被segment覆盖的传感器
    void addSensor(int index);
    void setRight(int r);
    void setActiveDistance(int dis);
    void setActiveTime(double time);
    // overload "<"
    bool operator< (const Segment& _segment) const;
};

class SSFSolverDisc {
private:
    ProblemDisc1D* problem;
    int sensorNum;
    Solution* solution;
    double cost;

public:
    SSFSolverDisc(ProblemDisc1D* prob);
    // 析构函数，只释放了solution的内存，没释放problem的
    ~SSFSolverDisc();
    Solution* getSolution() const;
    // 求解 1D 问题，结果存于 solution
    void solve();
    // 计算速度调度能耗
    void calCost();
    double getCost() const;

private:
    // 初始化
    void init(vector<ssf::Sensor> &sensors);
    // 寻找最慢段
    Segment findSlowestSegment(const vector<bool>& isActDis, const vector<Sensor>& sensors) const;
    // 找到最慢段后，更新各参数
    void update(const Segment& seg, vector<bool>& isActDis, vector<Sensor>& sensors, int& count);
    // 计算某区间（段）的active distance
    int getActiveDistance(int l, int r, const vector<bool>& isActDis) const;
};

}

#endif