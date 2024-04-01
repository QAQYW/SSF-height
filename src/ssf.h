#ifndef SSF_H
#define SSF_H

#include <algorithm>

#include "resource.h"
// #include "problemDisc1D.h"
// #include "problemDisc2D.h"

// 前置声明
class ProblemDisc1D;
class ProblemDisc2D;

namespace ssf {


/// @brief Slowest Segment First 求得的解（速度调度）
class Solution {

private:
    std::vector<double> speedSche;

public:
    Solution() {};
    // 构造函数，速度默认初始化为 resource::V_STAR
    Solution(int size);
    // 构造函数，速度初始化为 v
    Solution(int size, double v);
    // 改变某位置的速度
    void changeSpeedSche(int index, double v);
    // 获得速度调度
    std::vector<double> getSpeedSche() const;
    // 计算速度调度的能耗
    double calCost() const;
};

class Sensor {

private:
    bool active;        // 是否活跃
    int sensorIndex;    // 传感器在问题中的原编号
    int leftIndex;      // 范围左端点的位置
    int rightIndex;     // 范围右端点的位置

public:
    /// @brief 构造函数
    /// @param i 传感器编号
    /// @param l 左端点
    /// @param r 右端点
    Sensor(int i, int l, int r): active(true), sensorIndex(i), leftIndex(l), rightIndex(r) {};
    /// @brief 若活跃，则返回true
    bool isActive() const;
    /// @brief 将传感器设为活跃（active=true）
    void setActive();
    /// @brief 将传感器设为不活跃（active=false）
    void setInactive();
    int getSensorIndex() const;
    int getLeftIndex() const;
    int getRightIndex() const;
    /// @brief 重载"<"运算符，先按 leftIndex 升序，再按 rightIndex 升序
    bool operator< (const Sensor& _sensor) const;
};

class Segment {

private:
    int left;
    int right;
    int activeDistance;
    double activeTime;
    double velocity;
    /// @brief 该段（segment）对应的传感器集合
    std::vector<int> sensorList;

public:
    /// @brief 构造函数，速度自动设为 d / t
    /// @param l 左端点
    /// @param r 右端点
    /// @param d 活跃距离
    /// @param t 活跃时间（未被采集数据的传感器的所需传输时间之和）
    Segment(int l, int r, int d, double t);
    double getVelocity() const;
    int getLeft() const;
    int getRight() const;
    int getActiveDistance() const;
    double getActiveTime() const;
    std::vector<int> getSensorList() const;
    /// @brief 根据activeDistance和activeTime来计算真实速度
    void calVelocity();
    /// @brief 增加新发现的被segment覆盖的传感器，直接添加到sensorList的末尾
    /// @param index 是对sensors排序后的索引，而非传感器编号sensorIndex
    void addSensor(int index);
    void setRight(int r);
    void setActiveDistance(int dis);
    void setActiveTime(double time);
    void setVelocity(double v);
    /// @brief 重载"<"运算符。按优先级从高到低分别是速度velocity升序、覆盖的传感器数量sensorList.size()升序，左端点left升序，右端点right升序
    bool operator< (const Segment& _segment) const;
    /**
     * Online
    */
    /// @brief 增加新发现的被segment覆盖的传感器，按右端点rightIndex升序插入到sensorList
    /// @param index 新增的传感器在sensors中的索引
    /// @param sensors 传感器的信息
    void addSensorWithOrder(int index, const std::vector<ssf::Sensor> &sensors);
};

class SSFSolverDisc {
private:
    const ProblemDisc1D *problem;
    const ProblemDisc2D *problemFrom;
    int sensorNum;
    Solution solution;
    double cost;

public:
    /// @brief 构造函数（problemFrom默认为nullptr）
    /// @param prob 一维离散问题
    SSFSolverDisc(const ProblemDisc1D *prob);
    /// @brief 构造函数，求解在线问题时用
    /// @param prob 一维离散问题
    /// @param from 二维离散问题，prob由from转换而来
    SSFSolverDisc(const ProblemDisc1D* prob, const ProblemDisc2D* from);
    Solution getSolution() const;
    void solve();
    void calCost();
    double getCost() const;

private:
    
    /// @brief 初始化所有传感器，并排序
    /// @param sensors 空的传感器集合
    void init(std::vector<ssf::Sensor> &sensors);
    // 寻找最慢段
    Segment findSlowestSegment(const std::vector<bool> &isActDis, const std::vector<ssf::Sensor> &sensors) const;
    
    /// @brief 找到最慢段后，更新各参数，包括设置速度
    /// @param seg 最慢段
    /// @param isActDis 距离活跃标记
    /// @param sensors 传感器集合
    /// @param count 目前活跃的传感器数量
    void update(const Segment& seg, std::vector<bool>& isActDis, std::vector<ssf::Sensor>& sensors, int& count);
    /// @brief 计算某段区间的active distance（离散）
    /// @param l 区间左端点
    /// @param r 区间右端点
    /// @param isActDis 距离活跃标记
    /// @return 离散的active distance长度
    int getActiveDistance(int l, int r, const std::vector<bool>& isActDis) const;

/**
 * Online
*/
public:
    void solveForOnline(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked);

private:
    Segment findSlowestSegmentForOnline(const std::vector<bool> &isActDis, const std::vector<ssf::Sensor> &sensors) const;
};

} // namespace ssf

#endif