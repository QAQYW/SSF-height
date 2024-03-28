#ifndef ACO_ONLINE_H
#define ACO_ONLINE_H

/* -------------------------------------------------------------------------- */
/*                            ACO (online version)                            */
/* -------------------------------------------------------------------------- */

#include "aco.h"
#include "problemOnlineDisc2D.h"

namespace online {

class Sensor {
private:
    int sensorIndex;     // 传感器原来的编号
    double time;         // 剩余时间
    // int leftIndex;       // 数据传输范围的左端点
    // int rightIndex;      // 数据传输范围的右端点
    // int controlBoundary; // 控制通信范围的左端点（右端点用不到）
public:
    Sensor(int i, double t): sensorIndex(i), time(t) {};
};

class ACOSolver_Online {

private:
    ProblemOnlineDisc2D *problem; // 要解决的问题
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    aco::Trajectory trajectory; // 路径
    vector<int> rBound;         // 辅助变量
    vector<online::Sensor> sensors; // 传感器集合

public:
    ACOSolver_Online(ProblemOnlineDisc2D *prob);
    ProblemOnlineDisc2D* getProblem() const;
    aco::Trajectory getTrajectory() const;
    void solve();
    void resolve(int start, int end); // 每次获得新信息的时候调用
};



}



#endif