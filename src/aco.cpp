#include "aco.h"

/* -------------------------------- parameter ------------------------------- */

const int aco::ANT_NUM = 30;    // ! 未调参
const double aco::ALPHA = 5; //1;    // ! 未调参
const double aco::BETA = 1; //6;     // ! 未调参
const double aco::INITIAL_PHEROMONE_VALUE = 1;  // ! 未调参
const double aco::EVAPORATE_COEF = 0.2;         // ! 未调参
const double aco::ENHANCE_VALUE = 1; //0.5;          // ! 未调参
const int aco::MAX_INTERATOR = 10; //50;              // ! 未调参
const double aco::HEURISTIC_BASE = 1;           // ? 这个值具体是多少好像不重要
const double aco::HEURISTIC_REDUCE_FACTOR = 1;  // ! 慎重取值。和无人机升降的能耗有直接关系

/* ----------------------------- basic function ----------------------------- */

// 返回的是高度heightIndex，而不是对应项在candList中的索引
// TODO 可以用二分
int aco::roulette(vector<aco::Candidate>& candList, double sum) {
    if (candList.empty()) {
        cout << "Roulette Error: candList is empty\n";
        return 0;
    }
    double accu = 0;
    double r = tools::randDouble(0, sum);
    int num = candList.size();
    for (int i = 0; i < num; i++) {
        accu += candList[i].p;
        if (r < accu) {
            return candList[i].h;
        }
    }
    // cout << "Roulette Error.\n";
    // cout << "randomValue = " << r << "\n";
    // cout << "sumProbability = " << sum << "\n";
    return candList.back().h;
}


/* -------------------------------- ACOSolver ------------------------------- */

aco::ACOSolver::ACOSolver(ProblemDisc2D* prob): problem(prob), trajectory() {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
    // 构造 lBound 和 rBound
    // lBound.resize(lengthIndexNum, 0);
    rBound.resize(lengthIndexNum, 0);
    for (resource::SensorDisc2D s : problem->getSensorList()) {
        // int lMost = s.rangeList[0].leftIndex;
        int rMost = s.rangeList[0].rightIndex;
        for (resource::RangeDisc rg : s.rangeList) {
            // lMost = min(lMost, rg.leftIndex);
            rMost = max(rMost, rg.rightIndex);
        }
        // lBound[lMost]++;
        rBound[rMost]++;
    }
    for (int i = 1; i < lengthIndexNum; i++) {
        // lBound[i] += lBound[i - 1];
        rBound[i] += rBound[i - 1];
    }
    // this->trajectory = nullptr;
}

// aco::ACOSolver::~ACOSolver() {
//     if (trajectory != nullptr) {
//         delete trajectory;
//     }
// }

// TODO: complete
// ? 暂时用不到了
// void aco::ACOSolver::copyTrajectory(const aco::Trajectory &traj) {
//     vector<int> list = traj.getHeightSche();
//     int len = list.size();
//     if (trajectory != nullptr) {
//         delete trajectory;
//     }
//     trajectory = new Trajectory(len);
//     for (int i = 0; i < len; i++) {
//         trajectory->setHeightIndex(list[i], i);
//     }
// }

ProblemDisc2D* aco::ACOSolver::getProblem() const {
    return problem;
}

aco::Trajectory aco::ACOSolver::getTrajectory() const {
    return trajectory;
}

int aco::ACOSolver::getSensorNum() const {
    return sensorNum;
}

// int aco::ACOSolver::getLBoundValue(int index) const {
//     return lBound[index];
// }

int aco::ACOSolver::getRBoundValue(int index) const {
    return rBound[index];
}

void aco::ACOSolver::solve() {
    // 信息素矩阵的维度
    vector<int> dim(3, 0);
    dim[0] = problem->getLengthDiscNum();
    dim[1] = dim[2] = problem->getHeightDiscNum();
    // 定义信息素矩阵
    // pheromone[d][h1][h2] 表示从(d-1,h1)到(d,h2)的信息素强度
    // 起点时视为从 (dummy start point, minHeightIndex) -> (0, sche[0])
    // 初始化为 aco::INITIAL_PHEROMONE_VALUE
    vector<vector<vector<double>>> pheromone(
        dim[0],
        vector<vector<double>>(
            dim[1],
            vector<double> (dim[2], aco::INITIAL_PHEROMONE_VALUE)
        )
    );

    // 所有蚂蚁集合
    vector<Ant> ants(aco::ANT_NUM, Ant());

    // 以固定高度 minHeightIndex 飞行的轨迹，作为初始解
    Ant bestAnt(problem->getLengthDiscNum(), problem->getMinHeightIndex());
    bestAnt.calCost(*problem);
    double optimalCost = bestAnt.getCost();
    // this->trajectory = bestAnt.getTrajectory();
    // copyTrajectory(*bestAnt.getTrajectory());
    trajectory = bestAnt.getTrajectory();

    int iter = 0;
    // TODO 也可以换成其他循环终止条件
    while (iter < aco::MAX_INTERATOR) {
        int bestIndex = 0;
        for (int i = 0; i < aco::ANT_NUM; i++) {
            ants[i].init(problem->getLengthDiscNum());
            ants[i].generateTrajectory(lengthIndexNum, pheromone, *this);
            ants[i].calCost(*problem);
            if (ants[i].getCost() < ants[bestIndex].getCost()) {
                bestIndex = i;
            }
        }
        if (ants[bestIndex].getCost() < bestAnt.getCost()) {
            bestAnt = ants[bestIndex];
        }
        evaporatePheromone(dim, pheromone);   // 蒸发
        enhancePheromone(bestAnt, pheromone); // 增强
        if (bestAnt.getCost() < optimalCost) {
            optimalCost = bestAnt.getCost();
            // this->trajectory = bestAnt.getTrajectory();
            // copyTrajectory(*bestAnt.getTrajectory());
            trajectory = bestAnt.getTrajectory();
        }
        ++iter;
        cout << "Iter: " << iter << "/" << aco::MAX_INTERATOR << "\n";
    }
    // cout << "Iteration over\n";
    ants.clear();
}

void aco::ACOSolver::evaporatePheromone(const vector<int>& dim, vector<vector<vector<double>>> &ph) const {
    double coef = 1.0 - aco::EVAPORATE_COEF;
    for (int i = 0; i < dim[0]; i++) {
        for (int j = 0; j < dim[1]; j++) {
            for (int k = 0; k < dim[2]; k++) {
                ph[i][j][k] *= coef;
            }
        }
    }
}

void aco::ACOSolver::enhancePheromone(const Ant& ant, vector<vector<vector<double>>> &ph) const {
    // TODO 增强信息素
    // trajectory 的长度是 lengthIndexNum
    // vector<int> sche = trajectory.getHeightSche();
    // ! 应该用bestAnt（即形参ant）的trajectory来更新
    vector<int> sche = ant.getTrajectory().getHeightSche();
    for (int i = 1; i < lengthIndexNum; i++) {
        ph[i][sche[i - 1]][sche[i]] += aco::ENHANCE_VALUE;
    }
    // ? 最小高度离散值
    // 起点高度默认从 (dummy start length, minHeightIndex) -> (0, sche[0])
    ph[0][problem->getMinHeightIndex()][sche[0]] += aco::ENHANCE_VALUE;
}

bool aco::ACOSolver::isUrgent(int d, vector<aco::Candidate> & candList, const vector<bool> &visit, int countVisit) const {
    candList.clear();
    // int dMax = d + sensorNum - countVisit;
    int dMax = std::min(d + sensorNum - countVisit, lengthIndexNum);
   // 0 ~ (d-1)的轨迹都已经确定
    for (int nd = d; nd < dMax; nd++) {
        // if (_d - d + 1 <= getRBoundValue(_d) - countVisit) {
        if (nd - d + 1 <= rBound[nd] - countVisit) {
            // whether urgent or not
            vector<bool> hFlag(heightIndexNum, false);
            int hMin = problem->getMinHeightIndex();
            int hMax = problem->getMaxHeightIndex();
            for (int i = 0; i < sensorNum; i++) { // i: sensor index
                resource::SensorDisc2D sensor = problem->getSensor(i);

                int hMost = std::min(hMax, (int) sensor.rangeList.size() - 1);

                int rMostIndex = 0;
                for (int h = hMin; h <= hMost; h++) {
                    rMostIndex = std::max(rMostIndex, sensor.rangeList[h].rightIndex);
                }
                // ? 未访问过，且在nd之前结束？忘了之前写的是什么意思了，反正下面这句if里的条件有问题
                // if (!visit[i] && rBound[i] <= nd) {
                if (!visit[i] && rMostIndex <= nd) {
                    // resource::SensorDisc2D sensor = problem->getSensor(i);
                    for (int h = hMin; h <= hMost; h++) {
                        // ? 下面这个if条件好像也有问题，仅仅d<=rightIndex可能还不够
                        //if (d <= sensor.rangeList[h].rightIndex) {
                        if (sensor.isCovered(d,h)) {
                            hFlag[h] = true;
                        }
                    }
                }
            }
            for (int h = hMin; h <= hMax; h++) {
                if (hFlag[h]) {
                    aco::Candidate cand;
                    cand.h = h;
                    candList.push_back(cand);
                }
            }
            return true;
        }
    }
    return false;
}

double aco::ACOSolver::calProbability(const vector<vector<vector<double>>> &ph, int d, int curr, int next) const {
    double tau = ph[d][curr][next];
    double eta = this->calHeuristic(d, curr, next);
    return std::pow(tau, aco::ALPHA) * std::pow(eta, aco::BETA);
}

double aco::ACOSolver::calHeuristic(int d, int curr, int next) const {
    int count = 0;
    for (int i = 0; i < sensorNum; i++) {
        if (problem->getSensor(i).isCovered(d, next)) {
            ++count;
        }
    }
    if (count == 0) {
        if (curr != next) {
            return 0;
        }
        return aco::HEURISTIC_BASE;
    }
    return count * aco::HEURISTIC_BASE / (1 + aco::HEURISTIC_REDUCE_FACTOR * std::abs(curr - next));
}

/* ----------------------------------- Ant ---------------------------------- */

// ? 两个构造函数都简化了，直接写在.h里面

// aco::Ant::Ant() {
//     trajectory = new Trajectory();
// }

// aco::Ant::Ant(int lengthDiscNum, int heightIndex) {
//     trajectory = new Trajectory(lengthDiscNum, heightIndex);
// }

// aco::Ant::~Ant() {
//     if (trajectory != nullptr) {
//         delete trajectory;
//     }
// }

double aco::Ant::getCost() const {
    return cost;
}

void aco::Ant::calCost(const ProblemDisc2D &problem) {
    cost = calHeightCost() + calSpeedCost(problem);
}

aco::Trajectory aco::Ant::getTrajectory() const {
    return trajectory;
}

void aco::Ant::init(int lengthDiscNum) {
    // if (trajectory != nullptr) {
    //     delete trajectory;
    // }
    // trajectory = new Trajectory(lengthDiscNum);
    trajectory.reInit(lengthDiscNum, 0);
}

void aco::Ant::generateTrajectory(int trajLen, const vector<vector<vector<double>>> &ph, const aco::ACOSolver &solver) {
    
    trajectory = Trajectory(trajLen);

    // 当前sensor i是否被访问过
    vector<bool> visit(solver.getSensorNum(), false);
    // 访问过的传感器数量(visit[i] == true)
    int countVisit = 0;
    int sensorNum = solver.getSensorNum();
    // 起点高度
    // 以 minHeightIndex 作为虚拟起点的高度 // ! 暂时先用这个代替
    int hMin = solver.getProblem()->getMinHeightIndex();
    int hMax = solver.getProblem()->getMaxHeightIndex();
    int curr = hMin;
    for (int d = 0; d < trajLen; d++) {
        // 确定(d,h)
        vector<aco::Candidate> candidateList;
        double probSum = 0;
        // TODO 每确定了一个height，就要更新visit vector
        // cout << "going to isUrgent()\n";
        if (solver.isUrgent(d, candidateList, visit, countVisit)) {
            // cout << "\t In urgent case: " << std::to_string(candidateList.size()) << "\n";
            for (aco::Candidate cand : candidateList) {
                cand.p = solver.calProbability(ph, d, curr, cand.h);
                probSum += cand.p;
            }
        } else {
            // cout << "\t NOT in urgent case\n";
            for (int h = hMin; h <= hMax; h++) {
                Candidate cand;
                cand.h = h;
                cand.p = solver.calProbability(ph, d, curr, h);
                candidateList.push_back(cand);
                probSum += cand.p;
            }
        }

        // 确定高度
        int next = aco::roulette(candidateList, probSum);
        // trajectory.addList(next);
        trajectory.setHeightIndex(d, next);
        curr = next; 

        // 更新visit
        // vector<resource::SensorDisc2D> sensorList = solver->getProblem()->getSensorList();
        for (int i = 0; i < sensorNum; i++) {
            if (visit[i]) continue;

            if (solver.getProblem()->getSensor(i).isCovered(d, curr)) {
                visit[i] = true;
                ++countVisit;
            }
        }
        
    }
}

double aco::Ant::calHeightCost() const {
    return trajectory.calHeightCost();
}

double aco::Ant::calSpeedCost(const ProblemDisc2D &problem) const {
    return trajectory.calSpeedCost(problem);
}


/* ------------------------------- Trajectory ------------------------------- */

aco::Trajectory::Trajectory(int size) {
    heightSche.resize(size, 0);
}

aco::Trajectory::Trajectory(int size, int heightIndex) {
    heightSche.resize(size, heightIndex);
}

void aco::Trajectory::reInit(int size, int heightIndex) {
    heightSche.resize(size, heightIndex);
}

void aco::Trajectory::setHeightIndex(int lengthIndex, int heightIndex) {
    heightSche[lengthIndex] = heightIndex;
}

void aco::Trajectory::addList(int heightIndex) {
    heightSche.push_back(heightIndex);
}

vector<int> aco::Trajectory::getHeightSche() const {
    return heightSche;
}

int aco::Trajectory::getHeightIndex(int lengthIndex) const {
    return heightSche[lengthIndex];
}

double aco::Trajectory::calHeightCost() const {
    int size = heightSche.size();
    double cost = 0;
    for (int i = 1; i < size; i++) {
        cost += resource::costByHeight(heightSche[i - 1], heightSche[i]);
    }
    return cost;
}

double aco::Trajectory::calSpeedCost(const ProblemDisc2D &problem2D) const {
    double cost = 0;
    ProblemDisc1D problem1D;
    problem1D.transformFromProblemDisc2D(problem2D, *this);
    ssf::SSFSolverDisc ssfSolver(&problem1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    return ssfSolver.getCost();
}

double aco::Trajectory::calSpeedCost(const ProblemDisc2D &problem2D, vector<double> &speedSche) const {
    double cost = 0;
    ProblemDisc1D problem1D;
    problem1D.transformFromProblemDisc2D(problem2D, *this);
    ssf::SSFSolverDisc ssfSolver(&problem1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    speedSche = ssfSolver.getSolution().getSpeedSche();
    return ssfSolver.getCost();
}