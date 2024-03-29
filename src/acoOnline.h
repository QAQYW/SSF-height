#ifndef ACO_ONLINE_H
#define ACO_ONLINE_H

/* -------------------------------------------------------------------------- */
/*                            ACO (online version)                            */
/* -------------------------------------------------------------------------- */

#include "aco.h"
#include "problemOnlineDisc2D.h"
#include "problemDisc2D.h"


namespace online {

class Sensor {
private:
    // int sensorIndex;     // 传感器原来的编号
    double time;         // 剩余时间
    // int leftIndex;       // 数据传输范围的左端点
    // int rightIndex;      // 数据传输范围的右端点
    // int controlBoundary; // 控制通信范围的左端点（右端点用不到）
    bool active;

public:
    // 构造函数，默认初始时是不活跃的（未被发现，active=false）
    // Sensor(int i, double t): sensorIndex(i), time(t), active(false) {};
    Sensor(double t): time(t), active(false) {};
    // 获得（剩余）所需的传输时间
    double getTime() const;
    // 查询是否活跃
    bool isActive() const;
    // 减去 dTime 的时间
    void reduceTime(double dTime);
    // 数据采集完成，清空剩余时间
    void clearTime();
    // 设为活跃状态（被发现）
    void setActive();
    // 设为不活跃状态（采集完成）
    void setInactive();
};

class ACOSolver_Online {

private:
    ProblemOnlineDisc2D *problem; // 要解决的问题
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    aco::Trajectory trajectory; // 路径
    vector<int> rBound;         // 辅助变量
    vector<online::Sensor> sensorState; // 传感器集合
    double cost;
    double hcost;
    double vcost;

public:
    ACOSolver_Online(ProblemOnlineDisc2D *prob);
    ProblemOnlineDisc2D* getProblem() const;
    aco::Trajectory getTrajectory() const;
    vector<online::Sensor> getSensorState() const;
    void solve();
    void resolve(int start, int end); // ? 每次获得新信息的时候调用
    // 在一段unitLength内以v的速度飞行，并收集linked中传感器的数据
    void collectData(const vector<int> &linked, double v);
    // 更新无人机的能耗（仅hcost, vcost）
    void updateEnergy(double v, int currh, int nexth);
    // 接收新的传感器信息（若有）
    vector<int> exploreNewSensor(int d, int h, const vector<resource::SensorOnlineDisc2D> &sensorList, vector<bool> &informed);
};

}

#endif