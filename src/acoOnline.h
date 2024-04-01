#ifndef ACO_ONLINE_H
#define ACO_ONLINE_H

/* -------------------------------------------------------------------------- */
/*                            ACO (online version)                            */
/* -------------------------------------------------------------------------- */

#include <vector>

// #include "problemDisc2D.h"
// #include "problemOnline2D.h"
// #include "problemOnlineDisc2D.h"
#include "aco.h"
#include "resource.h"
#include "trajectory.h"

namespace online {

/* --------------------------------- Sensor --------------------------------- */

/// @brief 描述传感器及其数据的状态
class Sensor {
private:
    double time; // 剩余所需的传输时间
    bool active; // 是否活跃
public:
    /// @brief 构造函数，传感器默认不活跃
    /// @param t 传输时间
    Sensor(double t): time(t), active(false) {};
    double getTime() const;
    bool isActive() const;
    /// @brief 减去 dTime 的时间
    /// @param dTime 减去的时间
    void reduceTime(double dTime);
    /// @brief 数据采集完成，清空剩余时间
    void clearTime();
    /// @brief 设为活跃状态（被无人机探索到）
    void setActive();
    /// @brief 设为不活跃状态（采集完成）
    void setInactive();
};

/* ---------------------------- ACOSolver_Online ---------------------------- */

class ACOSolver_Online {

private:
    ProblemOnlineDisc2D *problem; // 要解决的问题
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    Trajectory trajectory;   // 路径
    std::vector<int> rBound; // 辅助变量 // ? 好像没用到
    std::vector<online::Sensor> sensorState; // 传感器集合
    double cost;
    double hcost;
    double vcost;

public:
    ACOSolver_Online(ProblemOnlineDisc2D *prob);
    ProblemOnlineDisc2D* getProblem() const;
    Trajectory getTrajectory() const;
    std::vector<online::Sensor> getSensorState() const;
    double getCost() const;
    double getHcost() const;
    double getVcost() const;
    /// @brief 求解在线问题，并传出结果
    /// @param speedSche 速度调度结果从speedSche传出
    void solve(std::vector<double> &speedSche);
    /// @brief 除去已飞行的部分路径，将当前已知信息作为子问题求解，规划子问题的调度。每次获得新信息的时候调用。
    /// @param start 子问题起点
    /// @param end 子问题终点
    /// @param speedSche 速度调度
    /// @param linked 传感器连接方案
    void resolve(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked); // ? 每次获得新信息的时候调用
    /// @brief 在一段unitLength内匀速飞行，并收集linked中传感器的数据
    /// @param linked 传感器连接方案
    /// @param v 无人机飞行速度
    void collectData(const std::vector<int> &linked, double v);
    // 更新无人机的能耗（仅计算hcost, vcost）
    void updateEnergy(double v, int currh, int nexth);
    // 在 (currd,currh) 尝试接收新的传感器信息（若有）
    std::vector<int> exploreNewSensor(int d, int h, const std::vector<resource::SensorOnlineDisc2D> &sensorList, std::vector<bool> &informed);
};

} // namespace online




#endif