#include "ssf.h"
#include <iostream>
#include "problemDisc1D.h"
#include "problemDisc2D.h"
#include <cstring>

/* -------------------------------- Solution -------------------------------- */

ssf::Solution::Solution(int size) {
    speedSche.resize(size, resource::V_STAR);
}

ssf::Solution::Solution(int size, double v) {
    speedSche.resize(size, v);
}

void ssf::Solution::changeSpeedSche(int index, double v) {
    speedSche[index] = v;
}

std::vector<double> ssf::Solution::getSpeedSche() const {
    return speedSche;
}

double ssf::Solution::calCost() const {
    double cost = 0;
    for (double v : speedSche) {
        cost += resource::costByFly(resource::REF_UNIT_LENGTH, v);
    }
    return cost;
}

/* --------------------------------- Sensor --------------------------------- */

bool ssf::Sensor::isActive() const {
    return active;
}

void ssf::Sensor::setActive() {
    active = true;
}

void ssf::Sensor::setInactive() {
    active = false;
}

int ssf::Sensor::getSensorIndex() const {
    return sensorIndex;
}

int ssf::Sensor::getLeftIndex() const {
    return leftIndex;
}

int ssf::Sensor::getRightIndex() const {
    return rightIndex;
}

bool ssf::Sensor::operator< (const ssf::Sensor &_sensor) const {
    if (leftIndex != _sensor.getLeftIndex()) {
        return leftIndex < _sensor.getLeftIndex();
    }
    return rightIndex < _sensor.getRightIndex();
}

/* --------------------------------- Segment -------------------------------- */

ssf::Segment::Segment(int l, int r, int d, double t): left(l), right(r), activeDistance(d), activeTime(t) {
    // velocity = activeDistance / activeTime;
    calVelocity();
}

double ssf::Segment::getVelocity() const {
    return velocity;
}

int ssf::Segment::getLeft() const {
    return left;
}

int ssf::Segment::getRight() const {
    return right;
}

int ssf::Segment::getActiveDistance() const {
    return activeDistance;
}

double ssf::Segment::getActiveTime() const {
    return activeTime;
}

std::vector<int> ssf::Segment::getSensorList() const {
    return sensorList;
}

void ssf::Segment::calVelocity() {
    double actualDis = activeDistance * resource::REF_UNIT_LENGTH;
    velocity = actualDis / activeTime;
}

void ssf::Segment::addSensor(int index) {
    sensorList.push_back(index);
}

void ssf::Segment::setRight(int r) {
    right = r;
}

void ssf::Segment::setActiveDistance(int dis) {
    activeDistance = dis;
}

void ssf::Segment::setActiveTime(double time) {
    activeTime = time;
}

void ssf::Segment::setVelocity(double v) {
    velocity = v;
}

bool ssf::Segment::operator< (const Segment& _segment) const {
    if (velocity == _segment.getVelocity()) {
        if (sensorList.size() == _segment.getSensorList().size()) {
            if (left == _segment.getLeft()) {
                return right < _segment.getRight();
            }
            return left < _segment.getLeft();
        }
        return sensorList.size() < _segment.getSensorList().size();
    }
    return velocity < _segment.getVelocity();
}

void ssf::Segment::addSensorWithOrder(int index, const std::vector<ssf::Sensor> &sensors) {
    if (sensorList.empty()) {
        sensorList.push_back(index);
        return;
    }
    int r = sensors[index].getRightIndex();
    int pos = sensorList.size();
    while (pos && r < sensors[sensorList[pos - 1]].getRightIndex()) {
        --pos;
    }
    sensorList.insert(sensorList.begin() + pos, index);
}

/* ------------------------------ SSFSolverDisc ----------------------------- */

ssf::SSFSolverDisc::SSFSolverDisc(const ProblemDisc1D *prob): problem(prob), problemFrom(nullptr) {
    sensorNum = prob->getSensorNum();
    solution = ssf::Solution(prob->getLengthDiscNum());
}

ssf::SSFSolverDisc::SSFSolverDisc(const ProblemDisc1D* prob, const ProblemDisc2D* from): problem(prob), problemFrom(from) {
    // std::cout << "in func: 'ssf::SSFSolverDisc::SSFSolverDisc', sensorNum = " << std::to_string(prob->getSensorNum()) << "\n";
    sensorNum = prob->getSensorNum();
    solution = ssf::Solution(prob->getLengthDiscNum());
}

ssf::Solution ssf::SSFSolverDisc::getSolution() const {
    return solution;
}

void ssf::SSFSolverDisc::solve() {
    // 所有传感器集合
    std::vector<ssf::Sensor> sensors;
    // true 代表 active，数据未采集
    std::vector<bool> isActDis(problem->getLengthDiscNum(), true);
    // 对所有传感器初始化，并排序（先按leftIndex升序，再按rightIndex升序）
    this->init(sensors);

    // 剩余未传输传感器数量
    int countActiveSensor = sensorNum;
    while (countActiveSensor > 0) {
        ssf::Segment seg = findSlowestSegment(isActDis, sensors);
        // std::cout << "check the slowest segment\n";
        // // TODO 加个断点看看seg.velocity
        // std::cout << "velocity = " << std::to_string(seg.getVelocity()) << "\n";
        // std::cout << "{ ";
        // for (int s : seg.getSensorList()) {
        //     std::cout << std::to_string(s) << " ";
        // }
        // std::cout << "}\n";
        if (seg.getVelocity() >= resource::V_STAR) {
            // 剩余速度都设为 V_STAR
            int num = problem->getLengthDiscNum();
            for (int i = 0; i < num; i++) {
                if (isActDis[i]) solution.changeSpeedSche(i, resource::V_STAR);
            }
            break;
        }
        update(seg, isActDis, sensors, countActiveSensor);
    }
}

void ssf::SSFSolverDisc::calCost() {
    cost = solution.calCost();
}

double ssf::SSFSolverDisc::getCost() const {
    return cost;
}

void ssf::SSFSolverDisc::init(std::vector<ssf::Sensor> &sensors) {
    sensors.clear();
    // sensors.resize(problem->getSensorNum());
    for (int i = 0; i < sensorNum; i++) {
        int l = problem->getSensor(i).range.leftIndex;
        int r = problem->getSensor(i).range.rightIndex;
        if (l < 0 || r < 0) {
            std::cout << "error in init()\n";
        }
        ssf::Sensor sensor(i, l, r);
        // i是在problem中的编号，如果是online problem，还需要在获取解时映射一次编号
        sensors.push_back(sensor);
    }
    // 先按leftIndex升序排序
    // 若leftIndex相等，则按照rightIndex升序排序
    std::sort(sensors.begin(), sensors.end());
}

ssf::Segment ssf::SSFSolverDisc::findSlowestSegment(const std::vector<bool>& isActDis, const std::vector<ssf::Sensor>& sensors) const {
    // 所有segment集合
    std::vector<ssf::Segment> segments;
    // 辅助变量，是否被当前segment选中
    bool isChosen[isActDis.size()] = {false};
    for (int il = 0; il < sensorNum; il++) {
        // 若该传感器数据已传输，则跳过
        if (!sensors[il].isActive()) continue;

        std::memset(isChosen, false, sizeof(isChosen));

        // segment 的基本信息
        int lMost = sensors[il].getLeftIndex();  // segment的最左端
        int rMost = sensors[il].getRightIndex(); // segment的最右端
        // int dis = getActiveDistance(lMost, rMost, isActDis); // active distance
        int dis = getActiveDistance(sensors[il], isActDis, isChosen);
        // ? active distance 假如中间被挖空了呢
        double time = problem->getSensor(sensors[il].getSensorIndex()).time; // active time

        // 单独一个传感器的 segment 也要记录
        ssf::Segment seg(lMost, rMost, dis, time);
        seg.addSensor(il);
        seg.calVelocity();
        segments.push_back(seg);

        for (int ir = il + 1; ir < sensorNum; ir++) {
            // 若该传感器数据已传输，则跳过
            if (!sensors[ir].isActive()) continue;

            // 若该传感器与当前segment无重叠部分，则无法组成新的segment
            // int l = sensors[ir].getLeftIndex();
            // if (l > rMost) break;
            if (!isOverlap(sensors[ir].getSensorIndex(), isActDis, isChosen)) break; // 若无重叠则break

            // 更新 segment 的基本信息
            if (sensors[ir].getRightIndex() > rMost) {
                rMost = sensors[ir].getRightIndex();
                // dis += getActiveDistance(rMost + 1, sensors[ir].getRightIndex(), isActDis); // 更新 active distance
            }
            dis += getActiveDistance(sensors[ir], isActDis, isChosen);
            time += problem->getSensor(sensors[ir].getSensorIndex()).time; // 更新 active time
            // rMost = sensors[ir].getRightIndex(); // ! sensors[ir].getRightIndex()也可能比原rMost小（被包含）
            
            // 生成新的 segment 加入集合 segments 中
            // ssf::Segment seg(lMost, rMost, dis, time);
            seg.setActiveDistance(dis);
            seg.setActiveTime(time);
            seg.setRight(rMost);
            seg.addSensor(ir);
            seg.calVelocity();
            segments.push_back(seg);
        }
    }

    // sort(segments.begin(), segments.end());
    
    int index = 0; // the index of the slowest segment
    for (int i = segments.size() - 1; i; i--) {
        if (segments[i] < segments[index]) {
        // if (segments[i].getVelocity() < segments[index].getVelocity()) {
            index = i;
        }
    }

    return segments[index];
}

void ssf::SSFSolverDisc::update(const ssf::Segment& seg, std::vector<bool>& isActDis, std::vector<ssf::Sensor>& sensors, int& count) {
    // 确定slowest segment对应的速度，并更新active distance和active time
    double v = seg.getVelocity();
    // int l = seg.getLeft(), r = seg.getRight();
    // // TODO 这里要改，因为segment不连续，所以不能把[l,r]中所有distance都更新，应该加一个coverList
    // TODO 或者对Segment::sensorList中每个传感器的coverList都更新一遍
    // TODO 这样在更新时会有重复，但是在构造segment时就不需要花费额外的时间，也不用更改Segment的数据结构
    // for (int i = l; i <= r; i++) {
    //     if (isActDis[i]) {
    //         solution.changeSpeedSche(i, v);
    //         isActDis[i] = false;
    //     }
    // }
    // TODO 改成了这样的，把下面for (int index : seg.getSensorList())这个循环也合并进来了
    for (int sid : seg.getSensorList()) {
        for (int d : problem->getSensor(sensors[sid].getSensorIndex()).coverList) {
            if (isActDis[d]) {
                solution.changeSpeedSche(d, v);
                isActDis[d] = false;
            }
        }
        sensors[sid].setInactive();
    }
    // for (int index : seg.getSensorList()) {
    //     sensors[index].setInactive();
    // }
    count -= seg.getSensorList().size();
}

/// @brief 计算某段区间的active distance（离散）
/// @param l 区间左端点
/// @param r 区间右端点
/// @param isActDis 距离活跃标记
/// @return 离散的active distance长度
int ssf::SSFSolverDisc::getActiveDistance(int l, int r, const std::vector<bool>& isActDis) const {
    // ! 这个函数已经弃用了
    int dis = 0;
    for (int i = l; i <= r; i++)
        dis += isActDis[i] ? 1 : 0;
    return dis;
}

/// @brief 计算某段区间的active distance（离散），考虑了不连续的传输范围
/// @param sensor 新增进segment中的传感器
/// @param isActDis 距离活跃标记
/// @param isChosen 距离是否被当前segment选中的标记
/// @return 离散的active distance长度
int ssf::SSFSolverDisc::getActiveDistance(const ssf::Sensor &sensor, const std::vector<bool> &isActDis, bool isChosen[]) const {
    int dis = 0;
    for (int d : this->problem->getSensor(sensor.getSensorIndex()).coverList) {
        if (isActDis[d] && !isChosen[d]) {
            ++dis;
            isChosen[d] = true;
        }
    }
    return dis;
}

bool ssf::SSFSolverDisc::isOverlap(int sid, const std::vector<bool> &isActDis, const bool isChosen[]) const {
    for (int d : problem->getSensor(sid).coverList) {
        if (isActDis[d] && isChosen[d]) {
        // if (isActDis[d] || isChosen[d]) {
            /* 
                if 里的isActDis[d]应该是多余的
                因为isChosen[d]==true的地方一定有isActDis[d]==false
                详见 getActiveDistance() 函数
                为了保险还是加上去
            */
            return true;
        }
    }
    return false;
}

void ssf::SSFSolverDisc::solveForOnline(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked) {
    // std::cout << "in func: ssf::SSFSolverDisc::solveForOnline(...)\n";
    
    // 所有传感器集合
    std::vector<ssf::Sensor> sensors;
    // true 代表 active，数据未采集
    std::vector<bool> isActDis(problem->getLengthDiscNum(), true);
    // 对所有传感器初始化，并排序
    this->init(sensors);

    // 把原来的linked清空，否则一直push_back()累积
    for (int i = start; i < end; i++) { // i <= end
        linked[i].clear();
    }

    // 剩余未传输传感器数量
    int countActiveSensor = sensorNum;
    // std::cout << "number of sensors is " << std::to_string(sensorNum) << "\n";
    while (countActiveSensor > 0) {
        ssf::Segment seg = findSlowestSegmentForOnline(isActDis, sensors);
        // std::cout << "check the slowest segment (for online)\n";
        // // TODO 加个断点看看seg.velocity
        // std::cout << "velocity = " << std::to_string(seg.getVelocity()) << "\n";
        // std::cout << "{ ";
        // for (int s : seg.getSensorList()) {
        //     std::cout << std::to_string(s) << " ";
        // }
        // std::cout << "}\n";
        if (seg.getVelocity() >= resource::V_STAR) {
            // 这里的sensorList是剩余所有传感器的集合
            int l = 0, r = problem->getLengthDiscNum() - 1;
            int dis = getActiveDistance(l, r, isActDis);
            // ? 这里的dis是多少已经不重要，随便什么值都不影响结果
            // ? 因为active distance是为了计算速度，而下面直接setVelocity()了
            ssf::Segment segStar(l, r, dis, 0);
            segStar.setVelocity(resource::V_STAR);
            // ? 同理，下面这一段求active time也没有必要
            double time = 0;
            for (int i = 0; i < sensorNum; i++) {
                if (sensors[i].isActive()) {
                    time += problem->getSensor(sensors[i].getSensorIndex()).time;
                    segStar.addSensorWithOrder(i, sensors);
                }
            }
            segStar.setActiveTime(time);
            // 最后还要再update一下，设置V_STAR的速度
            update(segStar, isActDis, sensors, countActiveSensor);
            // for (int i = 0; i < problem->getLengthDiscNum(); i++) {
            //     if (isActDis[i]) {
            //         for (int j = 0; j < problem->getSensorNum(); j++) {
            //             if (sensors[j].isActive() && problem->getSensor(j).)
            //             linked[i + start].push_back(problemFrom->mapSensor(sensors[j].getSensorIndex()));
            //         }
            //     }
            // }
            for (int j = 0; j < problem->getSensorNum(); j++) {
                if (!sensors[j].isActive()) continue;
                for (int i : problem->getSensor(j).coverList) {
                    if (isActDis[i]) {
                        linked[i + start].push_back(problemFrom->mapSensor(sensors[j].getSensorIndex()));
                    }
                }
            }
            break;
        }

        // 将所连接的传感器结果传出
        int tempr = seg.getRight();
        std::vector<int> list = seg.getSensorList();
        int tempcnt = list.size();
        for (int i = seg.getLeft(); i <= tempr; i++) {
            if (i < 0) {
                std::cout << "check segment";
                std::cout << "\n";
            }
            if (!isActDis[i]) continue;
            for (int j = 0; j < tempcnt; j++) {
                // if (sensors[list[j]].getLeftIndex() <= i && sensors[list[j]].getRightIndex() >= i) {
                // ? 把if条件改为下面这个（左闭右开 区间）
                if (sensors[list[j]].getLeftIndex() <= i && i < sensors[list[j]].getRightIndex()) {
                    // int offlineIndex = sensors[list[j]].getSensorIndex();
                    // int onlineIndex = problemFrom->mapSensor(offlineIndex);
                    // linked[i + start].push_back(onlineIndex);
                    linked[i + start].push_back(problemFrom->mapSensor(sensors[list[j]].getSensorIndex()));
                }
            }
        }
        // for (int sid : seg.getSensorList()) {
        //     int originIndex = problemFrom->mapSensor(sensors[sid].getSensorIndex());
        //     for (int d : problem->getSensor(sid).coverList) {
        //         if (isActDis[d]) {
        //             linked[d + start].push_back(originIndex);
        //         }
        //     }
        // }
        // for (int sid : seg.getSensorList()) {
        //     for (int d : problem->getSensor(sid).coverList) {
        //         isActDis[d] = true;
        //     }
        // }

        // 更新状态
        update(seg, isActDis, sensors, countActiveSensor);
    }

    // 将速度调度结果传出
    std::vector<double> sche = solution.getSpeedSche();
    for (int i = start; i < end; i++) { // i <= end
        // if (i - start >= sche.size()) {
        //     std::cout << "ssf::SSFSolverDisc::solveOnline()\n";
        // } else speedSche[i] = sche[i - start];
        speedSche[i] = sche[i - start];
    }
}

ssf::Segment ssf::SSFSolverDisc::findSlowestSegmentForOnline(const std::vector<bool> &isActDis, const std::vector<ssf::Sensor> &sensors) const {
    // 所有segment集合
    std::vector<ssf::Segment> segments;
    // 辅助变量，是否被当前segment选中
    bool isChosen[isActDis.size()] = {false};
    for (int il = 0; il < sensorNum; il++) {
        // 若该传感器数据已传输，则跳过
        if (!sensors[il].isActive()) continue;

        std::memset(isChosen, false, sizeof(isChosen));

        // segment 的基本信息
        int lMost = sensors[il].getLeftIndex();  // segment的最左端
        int rMost = sensors[il].getRightIndex(); // segment的最右端
        // int dis = getActiveDistance(lMost, rMost, isActDis); // active distance
        int dis = getActiveDistance(sensors[il], isActDis, isChosen);
        double time = problem->getSensor(sensors[il].getSensorIndex()).time; // active time

        // 单独一个传感器的 segment 也要记录
        ssf::Segment seg(lMost, rMost, dis, time);
        seg.addSensor(il);
        seg.calVelocity();
        segments.push_back(seg);

        for (int ir = il + 1; ir < sensorNum; ir++) {
            // 若该传感器数据已传输，则跳过
            if (!sensors[ir].isActive()) continue;

            // 若该传感器与当前segment无重叠部分，则无法组成新的segment
            // if (sensors[ir].getLeftIndex() > rMost) break;
            if (!isOverlap(sensors[ir].getSensorIndex(), isActDis, isChosen)) break; // 若无重叠则break

            // 更新 segment 的基本信息
            if (sensors[ir].getRightIndex() > rMost) {
                rMost = sensors[ir].getRightIndex();
                // dis += getActiveDistance(rMost + 1, sensors[ir].getRightIndex(), isActDis); // 更新 active distance
            }
            dis += getActiveDistance(sensors[ir], isActDis, isChosen);
            time += problem->getSensor(sensors[ir].getSensorIndex()).time; // 更新 active time
            
            // 生成新的 segment 假如集合 segments 中
            seg.setActiveDistance(dis);
            seg.setActiveTime(time);
            seg.setRight(rMost);
            // seg.addSensor(ir);
            seg.addSensorWithOrder(ir, sensors);
            seg.calVelocity();
            segments.push_back(seg);
        }
    }

    if (segments.empty()) {
        std::cout << "empty segments set";
        std::cout << "\n";
    }

    int index = 0;
    for (int i = segments.size() - 1; i; i--)
        if (segments[i] < segments[index]) index = i;
    if (segments[index].getLeft() < 0) {
        std::cout << "error segment\n";
    }
    return segments[index];
}