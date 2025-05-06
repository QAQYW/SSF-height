#include "aco.h"

#include "problemDisc1D.h"
#include "problemDisc2D.h"

#include "greedy.h"
#include "greedy3.h"

#include "pso.h"

#include <cstdio>
#include <iostream>

/* -------------------------------- parameter ------------------------------- */

const int aco::ANT_NUM = 20; //30;
double aco::ALPHA = 1; //5; //1;
double aco::BETA = 0.5; //5; //6;
double aco::EVAPORATE_COEF = 0.05; // 0.1;
const double aco::ENHANCE_VALUE = 0.1; //0.3; //0.5;
const int aco::MAX_ITERATOR = 5; //20; //30; //50;
const double aco::HEURISTIC_BASE = 1;           // 这个值具体是多少不重要
const double aco::HEURISTIC_REDUCE_FACTOR = 0.1; //0.1; //1;
const double aco::INITIAL_PHEROMONE_VALUE = 1;

bool aco::HEURISTIC_FLAG = true;

/* -------------------------------- roulette -------------------------------- */

int aco::roulette(std::vector<aco::Candidate>& candList, double sum) {
    if (candList.empty()) {
        // std::cout << "Roulette Error: candList is empty\n";
        return 0; // 0 就是hMin
    }
    double accu = 0, r = tools::randDouble(0, sum);
    int num = candList.size();
    for (int i = 0; i < num; i++) {
        accu += candList[i].p;
        if (r < accu) return candList[i].h;
    }
    // cout << "Roulette Error.\n";
    // cout << "randomValue = " << r << "\n";
    // cout << "sumProbability = " << sum << "\n";
    return candList.back().h;
}

/* ----------------------------------- Ant ---------------------------------- */

aco::Ant::Ant() {
    trajectory = Trajectory();
}

aco::Ant::Ant(int lengthDiscNum, int heightIndex) {
    trajectory = Trajectory(lengthDiscNum, heightIndex);
}

aco::Ant::Ant(const Trajectory &traj) {
    trajectory = Trajectory(traj);
}

double aco::Ant::getCost() const {
    return cost;
}

void aco::Ant::calCost(const ProblemDisc2D &problem) {
    cost = calHeightCost() + calSpeedCost(problem);
}

Trajectory aco::Ant::getTrajectory() const {
    return trajectory;
}

void aco::Ant::init(int lengthDiscNum) {
    // if (trajectory != nullptr) {
    //     delete trajectory;
    // }
    // trajectory = new Trajectory(lengthDiscNum);
    trajectory.reInit(lengthDiscNum, 0);
}

void aco::Ant::generateTrajectory(int trajLen, const std::vector<std::vector<std::vector<double>>> &ph, const aco::ACOSolver &solver) {
    
    trajectory = Trajectory(trajLen); // init(trajLen);

    // 当前sensor i是否被访问过
    std::vector<bool> visit(solver.getSensorNum(), false);
    // 访问过的传感器数量(visit[i] == true)
    int countVisit = 0;
    int sensorNum = solver.getSensorNum();
    // 起点高度
    // 以 minHeightIndex 作为虚拟起点的高度
    int hMin = solver.getProblem()->getMinHeightIndex();
    int hMax = solver.getProblem()->getMaxHeightIndex();
    int curr = hMin;
    for (int d = 0; d < trajLen; d++) {
        // 确定(d,h)
        std::vector<aco::Candidate> candidateList;
        double probSum = 0;
        // 每确定了一个height，就要更新visit vector
        // cout << "going to isUrgent()\n";
        // if (d > 0 && solver.isUrgent(d, candidateList, visit, countVisit)) {
        if (solver.isUrgent(d, candidateList, visit, countVisit)) {
            // cout << "\t In urgent case: " << std::to_string(candidateList.size()) << "\n";
            for (aco::Candidate cand : candidateList) {
                cand.p = solver.calProbability(ph, d, curr, cand.h);
                probSum += cand.p;
            }
        } else {
            // cout << "\t NOT in urgent case\n";
            bool f[sensorNum];
            int cf = 0;
            for (int i = 0; i < sensorNum; i++) {
                f[i] = solver.getProblem()->getSensor(i).isCovered(d, curr);
                cf += f[i];
            }
            for (int h = hMin; h <= hMax; h++) {
                aco::Candidate cand;
                cand.h = h;
                // cand.p = solver.calProbability(ph, d, curr, h);
                cand.p = solver.calProbability(ph, d, curr, h, f, cf);
                candidateList.push_back(cand);
                probSum += cand.p;
            }
        }

        // std::cout << "check if candidate list is empty\n";
        // if (candidateList.empty()) {
        //     std::cout << "Empty candidate list";
        //     std::cout << "\n";
        // }

        

        // 确定高度
        // int next = (d == 0 ? hMin : aco::roulette(candidateList, probSum));
        // int next = aco::roulette(candidateList, probSum);
        int next = (candidateList.empty() ? curr : aco::roulette(candidateList, probSum));
        // trajectory.addList(next);
        trajectory.setHeightIndex(d, next);
        curr = next; 
        
        // if (d == 0) {
        //     for (aco::Candidate cd : candidateList) {
        //         std::cout << cd.h << ", ";
        //     }
        //     std::cout << std::endl << next << std::endl;
        //     if (next > 0) {
        //         char tempch = std::getchar();
        //     }
        // }

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

    // 手动释放
    // delete &visit;
}

double aco::Ant::calHeightCost() const {
    return trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR);
}

double aco::Ant::calSpeedCost(const ProblemDisc2D &problem) const {
    // return trajectory.calSpeedCost(problem);
    return energy_calculator::calSpeedCost(problem, trajectory);
}

/* -------------------------------- ACOSolver ------------------------------- */

aco::ACOSolver::ACOSolver(ProblemDisc2D *prob): problem(prob), trajectory() {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
    // 构造 rBound
    rBound.resize(lengthIndexNum + 10, 0); // ? 
    for (resource::SensorDisc2D s : problem->getSensorList()) {
        int rMost = s.rangeList[0].rightIndex;
        for (resource::RangeDisc rg : s.rangeList) {
            rMost = std::max(rMost, rg.rightIndex);
        }
        rBound[rMost]++;
    }
    for (int i = 1; i < lengthIndexNum; i++) {
        rBound[i] += rBound[i - 1];
    }
}

ProblemDisc2D* aco::ACOSolver::getProblem() const {
    return problem;
}

Trajectory aco::ACOSolver::getTrajectory() const {
    return trajectory;
}

int aco::ACOSolver::getSensorNum() const {
    return sensorNum;
}

void aco::ACOSolver::solve() {
    // 信息素矩阵的维度
    std::vector<int> dim(3, 0);
    dim[0] = problem->getLengthDiscNum();
    dim[1] = dim[2] = problem->getHeightDiscNum();
    // 定义三维信息素矩阵
    // pheromone[d][h1][h2] 表示从(d-1,h1)到(d,h2)的信息素强度
    // 起点时视为从 (dummy start point, minHeightIndex) -> (0, sche[0])
    // 初始化为 aco::INITIAL_PHEROMONE_VALUE
    std::vector<std::vector<std::vector<double>>> pheromone(
        dim[0],
        std::vector<std::vector<double>>(
            dim[1],
            std::vector<double> (dim[2], aco::INITIAL_PHEROMONE_VALUE)
        )
    );

    // 所有蚂蚁集合
    std::vector<aco::Ant> ants(aco::ANT_NUM, aco::Ant());
    double optimalCost;

    aco::Ant bestAnt(problem->getLengthDiscNum(), 0);

    // 随机初始化
    bool feasibleFlag = false; int countTrials = 0;
    while (!feasibleFlag) {
        Trajectory randomTraj(problem->getLengthDiscNum());
        for (int i = 0, tmpHeightIndex; i < problem->getLengthDiscNum(); i++) {
            tmpHeightIndex = tools::randInt(0, problem->getHeightDiscNum() - 1);
            randomTraj.setHeightIndex(i, tmpHeightIndex);
        }
        if (isFeasible(randomTraj)) {
            bestAnt = aco::Ant(randomTraj);
            bestAnt.calCost(*problem);
            optimalCost = bestAnt.getCost();
            trajectory = Trajectory(randomTraj);
            feasibleFlag = true;
        }
        ++countTrials;
    }
    // printf("\ntrials = %d\n\n", countTrials);

    // // 用greedy方法来初始化蚁群（所有高度中选最优的，以固定高度飞行）
    // greedy::GreedySolver greedySolver = greedy::GreedySolver(problem);
    // greedySolver.solve();
    // int initHeight = greedySolver.getTrajectory().getHeightIndex(0);
    // // initHeight = 0;
    // // 以固定高度 minHeightIndex 飞行的轨迹，作为初始解
    // bestAnt = aco::Ant(problem->getLengthDiscNum(), initHeight);
    // bestAnt.calCost(*problem);
    // optimalCost = bestAnt.getCost();
    // trajectory = greedySolver.getTrajectory();


    // 用greedy3初始化
    // greedy3::GreedySolver3 greedySovler3 = greedy3::GreedySolver3(problem);
    // greedySovler3.solve();
    // if (greedySovler3.getCost() < optimalCost) {
    //     bestAnt = aco::Ant(greedySovler3.getTrajectory());
    //     trajectory = greedySovler3.getTrajectory();
    //     bestAnt.calCost(*problem);
    //     optimalCost = bestAnt.getCost();
    // }

    int iter = 0;
    std::vector<double> optimalCostList;
    optimalCostList.push_back(optimalCost);
    // 也可以换成其他循环终止条件
    while (iter < aco::MAX_ITERATOR) {
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
            // std::cout << "better solution\n";
            bestAnt = ants[bestIndex];
            // if (emptyTraj) {
            //     optimalCost = bestAnt.getCost();
            //     trajectory = bestAnt.getTrajectory();
            //     emptyTraj = false;
            // }
        }
        evaporatePheromone(dim, pheromone);   // 蒸发
        enhancePheromone(bestAnt, pheromone); // 增强
        if (bestAnt.getCost() < optimalCost) {
            optimalCost = bestAnt.getCost();
            trajectory = bestAnt.getTrajectory();
        }
        ++iter;

        // if (iter == aco::MAX_ITERATOR - 1) {
        //     // todo 输出信息素浓度矩阵，然后可视化
        //     savePheromone(pheromone, ".\\newnewexp\\exp000");
        //     std::puts("-----  Now check pheromone matrix  -----");
        //     std::puts("");
        // }

        // if (iter == aco::MAX_ITERATOR - 1 || iter == 1) {
        //     // 输出信息素浓度矩阵，然后可视化
        //     savePheromone(pheromone, ".\\newnewexp\\exp_iter");
        //     std::puts("-----  Now check pheromone matrix  -----");
        //     std::puts("");
        // }

        // todo 改成 bestAnt.getCost() 的曲线图
        // optimalCostList.push_back(bestAnt.getCost());
        // optimalCostList.push_back(optimalCost);

        // todo 输出trajectory
        // std::vector<int> heightSche = bestAnt.getTrajectory().getHeightSche();+
        // // tools::printVector("Traj", ".\\newnewexp\\exp_iter\\iter_results.txt", heightSche); // save to file
        // puts("Traj:");
        // for (int _hei : heightSche) printf("%d, ", _hei);
        // puts("");

        // std::printf("iter: %d/%d, cost=%lf\n", iter, aco::MAX_ITERATOR, optimalCost);
    }
    ants.clear();

    // 输出 optimalCostList 到文件
    // tools::printVector("ACO", ".\\newnewexp\\exp_iter\\iter_results.txt", optimalCostList);
}

void aco::ACOSolver::evaporatePheromone(const std::vector<int>& dim, std::vector<std::vector<std::vector<double>>> &ph) const {
    // 信息素保留下来的系数
    double coef = 1.0 - aco::EVAPORATE_COEF;
    for (int i = 0; i < dim[0]; i++) {
        for (int j = 0; j < dim[1]; j++) {
            for (int k = 0; k < dim[2]; k++) {
                ph[i][j][k] *= coef;
            }
        }
    }
}

void aco::ACOSolver::enhancePheromone(const aco::Ant &ant, std::vector<std::vector<std::vector<double>>> &ph) const {
    // trajectory 的长度是 lengthIndexNum
    // vector<int> sche = trajectory.getHeightSche();
    // 应该用bestAnt（即形参ant）的trajectory来更新
    std::vector<int> sche = ant.getTrajectory().getHeightSche();
    for (int i = 1; i < lengthIndexNum; i++) {
        ph[i][sche[i - 1]][sche[i]] += aco::ENHANCE_VALUE;
    }
    // 起点高度默认从 (dummy start length, minHeightIndex) -> (0, sche[0])
    ph[0][problem->getMinHeightIndex()][sche[0]] += aco::ENHANCE_VALUE;
}

bool aco::ACOSolver::isUrgent(int d, std::vector<aco::Candidate> & candList, const std::vector<bool> &visit, int countVisit) const {
    
    candList.clear();
    // int dMax = d + sensorNum - countVisit;
    int dMax = std::min(d + sensorNum - countVisit, lengthIndexNum);
   // 0 ~ (d-1)的轨迹都已经确定
    for (int nd = d + 1; nd <= dMax; nd++) { // nd: next d
        // 计算真正的countVisit
        countVisit = 0;
        for (int i = 0; i < sensorNum; i++) {
            if (visit[i] && problem->getSensor(i).rmost <= nd) {
                ++countVisit;
            }
        }
        if (nd - d <= rBound[nd] - countVisit) {
            // 是否urgent
            std::vector<bool> hFlag(heightIndexNum, false);
            int hMin = problem->getMinHeightIndex();
            int hMax = problem->getMaxHeightIndex();
            for (int i = 0; i < sensorNum; i++) { // i: sensor index
                resource::SensorDisc2D sensor = problem->getSensor(i);

                int hMost = std::min(hMax, (int) sensor.rangeList.size() - 1);

                int rMostIndex = 0;
                for (int h = hMin; h <= hMost; h++) {
                    rMostIndex = std::max(rMostIndex, sensor.rangeList[h].rightIndex);
                }
                if (!visit[i] && rMostIndex <= nd) {
                    for (int h = hMin; h <= hMost; h++) {
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

double aco::ACOSolver::calProbability(const std::vector<std::vector<std::vector<double>>> &ph, int d, int curr, int next) const {
    double tau = ph[d][curr][next];
    double eta = this->calHeuristic(d, curr, next);
    return std::pow(tau, aco::ALPHA) * std::pow(eta, aco::BETA);
}

double aco::ACOSolver::calProbability(const std::vector<std::vector<std::vector<double>>> &ph, int d, int curr, int next, bool f[], int cf) const {
    double tau = ph[d][curr][next];
    double eta = this->calHeuristic(d, curr, next, f, cf);
    return std::pow(tau, aco::ALPHA) * std::pow(eta, aco::BETA);
}

double aco::ACOSolver::calHeuristic(int d, int curr, int next) const {
    int count = 0;
    for (int i = 0; i < sensorNum; i++) {
        if (problem->getSensor(i).isCovered(d, next)) ++count;
    }
    if (count == 0) {
        if (curr != next) return 0;
        return aco::HEURISTIC_BASE;
    }
    // if (curr == next) {
    //     return sensorNum * HEURISTIC_BASE;
    // }
    // return count * aco::HEURISTIC_BASE / (1 + aco::HEURISTIC_REDUCE_FACTOR * std::abs(curr - next));
    return count * aco::HEURISTIC_BASE;
}

double aco::ACOSolver::calHeuristic(int d, int curr, int next, bool f[], int cf) const {
    if (curr == next) {
        return std::max(cf, 1);
    }
    int cg = 0, ctemp = 0;
    bool g[sensorNum], flag = false;
    for (int i = 0; i < sensorNum; i++) {
        g[i] = problem->getSensor(i).isCovered(d, next);
        cg += g[i];
        ctemp += (f[i] & g[i]);
        flag |= (g[i] & !f[i]);
    }
    if (!aco::HEURISTIC_FLAG) return cg; // 不使用基于支配关系的筛选
    if (ctemp <= cg && !flag) return 0;
    return cg;
}

bool aco::ACOSolver::isFeasible(Trajectory traj) const {
    int num = problem->getSensorNum();
    bool collected[num] = {false};
    for (int sid = 0; sid < num; sid++) {
        int lmost = problem->getLengthDiscNum() - 1;
        int rmost = 0;
        for (resource::RangeDisc rg : problem->getSensor(sid).rangeList) {
            lmost = std::min(lmost, rg.leftIndex);
            rmost = std::max(rmost, rg.rightIndex);
        }
        for (int dis = lmost, hei; !collected[sid] && dis < rmost; dis++) {
            hei = traj.getHeightIndex(dis);
            if (problem->getSensor(sid).isCovered(dis, hei)) {
                collected[sid] = true;
            }
        }
        if (!collected[sid]) return false;
    }
    return true;
}

void aco::ACOSolver::solveForOnline(int start, int end, std::vector<double> &speedSche, std::vector<std::vector<int>> &linked) {
    /**
     * 刚开始都正常求解，最后才调用online版本的ssf
     * 把传感器连接方案、速度调度方案的结果传出
    */

    // 信息素矩阵的维度
    std::vector<int> dim(3);
    dim[0] = problem->getLengthDiscNum();
    dim[1] = dim[2] = problem->getHeightDiscNum();
    // 定义信息素矩阵：pheromone[d][h1][h2] 表示从(d-1,h1)到(d,h2)的信息素强度
    std::vector<std::vector<std::vector<double>>> pheromone(
        dim[0], std::vector<std::vector<double>>(
            dim[1], std::vector<double> (dim[2], aco::INITIAL_PHEROMONE_VALUE)
        )
    );

    // 所有蚂蚁集合
    std::vector<aco::Ant> ants(aco::ANT_NUM, aco::Ant());

    // 以固定高度 minHeightIndex 飞行的轨迹，作为初始解
    Ant bestAnt(problem->getLengthDiscNum(), problem->getMinHeightIndex());
    bestAnt.calCost(*problem);
    double optimalCost = bestAnt.getCost();
    trajectory = bestAnt.getTrajectory();

    int iter = 0;
    // while (iter < aco::MAX_ITERATOR) {
    int iterMax = aco::MAX_ITERATOR / 2;
    while (iter < iterMax) {
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
            trajectory = bestAnt.getTrajectory();
        }
        ++iter;
    }
    ants.clear();

    // 最后统一把结果传出
    ProblemDisc1D probDisc1D = ProblemDisc1D(problem->getSensorNum(), problem->getLength(), problem->getLengthDiscNum(), problem->getSensorList(), trajectory);
    ssf::SSFSolverDisc ssfSolver = ssf::SSFSolverDisc(&probDisc1D, problem);
    // 结果保存在speedSche和linked当中
    ssfSolver.solveForOnline(start, end, speedSche, linked);
}

void aco::ACOSolver::savePheromone(const std::vector<std::vector<std::vector<double>>> &ph, std::string dir) const {
    std::ofstream fout;
    fout.open(dir + "\\pheromone_copy.txt");
    int dim1 = ph.size();
    int dim2 = ph[0].size();
    int dim3 = ph[0][0].size();
    fout << dim1 << "\t" << dim2 << "\t" << dim3 << "\n";
    for (int i = 0; i < dim1; i++) {
        for (int j = 0; j < dim2; j++) {
            for (int k = 0; k < dim3; k++) {
                fout << ph[i][j][k] << "\t";
            }
            fout << "\n";
        }
    }
    fout.close();
}