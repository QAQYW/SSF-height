#include "ssf.h"

#include "problemDisc1D.h"
#include "problemDisc2D.h"

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
    // 对所有传感器初始化，并排序
    this->init(sensors);

    // 剩余未传输传感器数量
    int countActiveSensor = sensorNum;
    while (countActiveSensor > 0) {
        ssf::Segment seg = findSlowestSegment(isActDis, sensors);
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
        ssf::Sensor sensor(i, l, r);
        // i是在problem中的编号，如果是online problem，还需要在获取解时映射一次编号
        sensors.push_back(sensor);
    }
    // 先按leftIndex升序排序
    // 若leftIndex相等，则按照rightIndex升序排序
    std::sort(sensors.begin(), sensors.end());
}

ssf::Segment ssf::SSFSolverDisc::findSlowestSegment(const std::vector<bool>& isActDis, const std::vector<ssf::Sensor>& sensors) const {
    std::vector<ssf::Segment> segments;
    for (int il = 0; il < sensorNum; il++) {
        // 若该传感器数据已传输，则跳过
        if (!sensors[il].isActive()) continue;

        // segment 的基本信息
        int lMost = sensors[il].getLeftIndex();  // segment的最左端
        int rMost = sensors[il].getRightIndex(); // segment的最右端
        int dis = getActiveDistance(lMost, rMost, isActDis); // active distance
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
            int l = sensors[ir].getLeftIndex();
            if (l > rMost) break;

            // 更新 segment 的基本信息
            if (sensors[ir].getRightIndex() > rMost) {
                rMost = sensors[ir].getRightIndex();
                dis += getActiveDistance(rMost + 1, sensors[ir].getRightIndex(), isActDis); // 更新 active distance
            }
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
    int l = seg.getLeft(), r = seg.getRight();
    for (int i = l; i <= r; i++) {
        if (isActDis[i]) {
            solution.changeSpeedSche(i, v);
            isActDis[i] = false;
        }
    }
    for (int index : seg.getSensorList()) {
        sensors[index].setInactive();
    }
    count -= seg.getSensorList().size();
}

int ssf::SSFSolverDisc::getActiveDistance(int l, int r, const std::vector<bool>& isActDis) const {
    int dis = 0;
    for (int i = l; i <= r; i++)
        dis += isActDis[i] ? 1 : 0;
    return dis;
}

void ssf::SSFSolverDisc::solveForOnline(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked) {
    // 所有传感器集合
    std::vector<ssf::Sensor> sensors;
    // true 代表 active，数据未采集
    std::vector<bool> isActDis(problem->getLengthDiscNum(), true);
    // 对所有传感器初始化，并排序
    this->init(sensors);

    // 剩余未传输传感器数量
    int countActiveSensor = sensorNum;
    while (countActiveSensor > 0) {
        ssf::Segment seg = findSlowestSegmentForOnline(isActDis, sensors);
        if (seg.getVelocity() >= resource::V_STAR) {
            // 这里的sensorList是剩余所有传感器的集合
            int l = 0, r = problem->getLengthDiscNum() - 1;
            int dis = getActiveDistance(l, r, isActDis);
            ssf::Segment segStar(l, r, dis, 0);
            segStar.setVelocity(resource::V_STAR);
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
            break;
        }

        // 将所连接的传感器结果传出
        int tempr = seg.getRight();
        std::vector<int> list = seg.getSensorList();
        int tempcnt = list.size();
        for (int i = seg.getLeft(); i <= tempr; i++) {
            if (!isActDis[i]) continue;
            for (int j = 0; j < tempcnt; j++) {
                if (sensors[list[j]].getLeftIndex() <= i && sensors[list[j]].getRightIndex() >= i) {
                    // int offlineIndex = sensors[list[j]].getSensorIndex();
                    // int onlineIndex = problemFrom->mapSensor(offlineIndex);
                    // linked[i + start].push_back(onlineIndex);
                    linked[i + start].push_back(problemFrom->mapSensor(sensors[list[j]].getSensorIndex()));
                }
            }
        }

        // 更新状态
        update(seg, isActDis, sensors, countActiveSensor);
    }

    // 将速度调度结果传出
    std::vector<double> sche = solution.getSpeedSche();
    for (int i = start; i <= end; i++) {
        // if (i - start >= sche.size()) {
        //     std::cout << "ssf::SSFSolverDisc::solveOnline()\n";
        // } else speedSche[i] = sche[i - start];
        speedSche[i] = sche[i - start];
    }
}

ssf::Segment ssf::SSFSolverDisc::findSlowestSegmentForOnline(const std::vector<bool> &isActDis, const std::vector<ssf::Sensor> &sensors) const {
    std::vector<ssf::Segment> segments;
    for (int il = 0; il < sensorNum; il++) {
        // 若该传感器数据已传输，则跳过
        if (!sensors[il].isActive()) continue;

        // segment 的基本信息
        int lMost = sensors[il].getLeftIndex();  // segment的最左端
        int rMost = sensors[il].getRightIndex(); // segment的最右端
        int dis = getActiveDistance(lMost, rMost, isActDis); // active distance
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
            if (sensors[ir].getLeftIndex() > rMost) break;

            // 更新 segment 的基本信息
            if (sensors[ir].getRightIndex() > rMost) {
                rMost = sensors[ir].getRightIndex();
                dis += getActiveDistance(rMost + 1, sensors[ir].getRightIndex(), isActDis); // 更新 active distance
            }
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

    int index = 0;
    for (int i = segments.size() - 1; i; i--)
        if (segments[i] < segments[index]) index = i;
    return segments[index];
}