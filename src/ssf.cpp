#include "ssf.h"
// using namespace ssf;

/* ------------------------------ SSFSolverDisc ----------------------------- */

ssf::SSFSolverDisc::SSFSolverDisc(ProblemDisc1D* prob): problem(prob) {
    // problem = prob;
    sensorNum = prob->getSensorNum();
    // solution = new Solution(prob->getLengthDiscNum());
    this->solution = ssf::Solution(prob->getLengthDiscNum());
    // solution.reInit(prob->getLengthDiscNum(), resource::V_STAR);
}

// ssf::SSFSolverDisc::~SSFSolverDisc() {
//     if (solution != nullptr) {
//         delete solution;
//     }
// }

ssf::Solution ssf::SSFSolverDisc::getSolution() const {
    return solution;
}

void ssf::SSFSolverDisc::init(vector<ssf::Sensor> &sensors) {
    sensors.clear();
    // sensors.resize(problem->getSensorNum());
    for (int i = 0; i < sensorNum; i++) {
        int l = problem->getSensor(i).range.leftIndex;
        int r = problem->getSensor(i).range.rightIndex;
        ssf::Sensor sensor(i, l, r);
        sensors.push_back(sensor);
    }
    // 先按leftIndex升序排序
    // 若leftIndex相等，则按照rightIndex升序排序
    std::sort(sensors.begin(), sensors.end());
}

// !!!!!!!
void ssf::SSFSolverDisc::solve() {
    // TODO: complete
    vector<ssf::Sensor> sensors;
    // true 代表 active，未传输
    vector<bool> isActDis(sensorNum, true);
    // vector<double> speedSche(problem->getLengthDiscNum());

    this->init(sensors);
    int countActiveSensor = sensorNum; // 剩余未传输传感器数量
    while (countActiveSensor > 0) { // ! termination condition
        ssf::Segment seg = findSlowestSegment(isActDis, sensors);
        if (seg.getVelocity() < resource::V_STAR) {
            // set all empty speedSche as V_STAR
            int num = problem->getLengthDiscNum();
            for (int i = 0; i < num; i++) {
                if (isActDis[i]) {
                    solution.changeSpeedSche(i, resource::V_STAR);
                }
            }
            break;
        }
        update(seg, isActDis, sensors, countActiveSensor);
    }
}

ssf::Segment ssf::SSFSolverDisc::findSlowestSegment(const vector<bool>& isActDis, const vector<ssf::Sensor>& sensors) const {
    vector<ssf::Segment> segments;
    for (int il = 0; il < sensorNum; il++) {
        // 若该传感器数据已传输，则跳过
        if (!sensors[il].isActive()) continue;

        // segment 的基本信息
        int lMost = sensors[il].getLeftIndex();  // segment的最左端
        int rMost = sensors[il].getRightIndex(); // segment的最右端
        int dis = getActiveDistance(lMost, rMost, isActDis); // ! active distance 假如中间被挖空了呢？
        double time = problem->getSensor(sensors[il].getSensorIndex()).time; // active time

        // 单独一个传感器的 segment 也要记录
        ssf::Segment seg(lMost, rMost, dis, time);
        seg.addSensor(il);
        segments.push_back(seg);

        for (int ir = il + 1; ir < sensorNum; ir++) {
            // 若该传感器数据已传输，则跳过
            if (!sensors[ir].isActive()) continue;

            // 若该传感器与当前segment无重叠部分，则无法组成新的segment
            int l = sensors[ir].getLeftIndex();
            if (l > rMost) break;

            // 更新 segment 的基本信息
            dis += getActiveDistance(rMost + 1, sensors[ir].getRightIndex(), isActDis); // 更新 active distance
            time += problem->getSensor(sensors[ir].getSensorIndex()).time; // 更新 active time
            rMost = sensors[ir].getRightIndex();
            // 新的 segment 加入集合 segments 中
            // ssf::Segment seg(lMost, rMost, dis, time);
            // ? 怎么提高效率
            // 不创建新的 segment，而是在之前的 segment 的基础上进行修改
            seg.setActiveDistance(dis);
            seg.setActiveTime(time);
            seg.setRight(rMost);
            seg.addSensor(ir);
            segments.push_back(seg);
        }
    }

    // sort(segments.begin(), segments.end());
    // 只找一个slowest segment的话没必要sort，O(n)比较即可
    
    int index = 0; // the index of the slowest segment
    for (int i = segments.size() - 1; i; i--) {
        // if (segments[i] < segments[index]) {
        if (segments[i].getVelocity() < segments[index].getVelocity()) {
            index = i;
        }
    }
    return segments[index];
}

void ssf::SSFSolverDisc::update(const ssf::Segment& seg, vector<bool>& isActDis, vector<ssf::Sensor>& sensors, int& count) {
    // TODO 确定slowest segment对应的速度，并更新active distance和active time
    double v = seg.getVelocity();
    // if (v >= resource::V_STAR) {
    //     return true;
    // }
    int l = seg.getLeft(), r = seg.getRight();
    for (int i = l; i <= r; i++) {
        if (isActDis[i]) {
            // speedSche[i] = v;
            solution.changeSpeedSche(i, v);
            isActDis[i] = false;
        }
    }
    for (int index : seg.getSensorList()) {
        sensors[index].setInactive();
    }
    count -= seg.getSensorList().size();
    // return false;
}

// ! 效率较低 O(r-l)
// 计算[l,r]区间内的 active distance 长度
int ssf::SSFSolverDisc::getActiveDistance(int l, int r, const vector<bool>& isActDis) const {
    int dis = 0;
    for (int i = l; i <= r; i++) {
        // isActDis[i]等于1(true)或0(false)
        dis += isActDis[i];
    }
    return dis;
}

void ssf::SSFSolverDisc::calCost() {
    cost = solution.calCost();
}

double ssf::SSFSolverDisc::getCost() const {
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

/**
 * 重载“<”
 * 先按 leftIndex 升序
 * 再按 rightIndex 升序
*/
bool ssf::Sensor::operator< (const ssf::Sensor& _sensor) const {
    if (leftIndex != _sensor.getLeftIndex()) {
        return leftIndex < _sensor.getLeftIndex();
    }
    return rightIndex < _sensor.getRightIndex();
}

/* --------------------------------- Segment -------------------------------- */

ssf::Segment::Segment(int l, int r, int d, double t): left(l), right(r), activeDistance(d), activeTime(t) {
    velocity = activeDistance / activeTime;
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

vector<int> ssf::Segment::getSensorList() const {
    return sensorList;
}

/**
 * 增加这个 segment 中所包含的传感器
 * ! 这个 index 应为排序后的 index，即在 sensors 中的索引，而非 sensorIndex
*/
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

/**
 * 重载“<”
 * 按 velocity 升序（最慢的在最前面）
*/
bool ssf::Segment::operator< (const Segment& _segment) const {
    return velocity < _segment.getVelocity();
}

/* -------------------------------- Solution -------------------------------- */

ssf::Solution::Solution(int size) {
    speedSche.resize(size, resource::V_STAR);
}

ssf::Solution::Solution(int size, double v) {
    speedSche.resize(size, v);
}

void ssf::Solution::changeSpeedSche(int index, int v) {
    speedSche[index] = v;
}

vector<double> ssf::Solution::getSpeedSche() const {
    return speedSche;
}

double ssf::Solution::calCost() const {
    double cost = 0;
    for (double v : speedSche) {
        // ! 暂时用 indexToLength(1) 表示 UNIT_LENGTH
        // TODO 最好还是定义一个 UNIT_LENGTH
        // cost += resource::costByFly(resource::indexToLength(1, 0, resource::REF_UNIT_LENGTH), v);
        cost += resource::costByFly(resource::REF_UNIT_LENGTH, v);
    }
    return cost;
}