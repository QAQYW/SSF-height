#include "ga.h"

#include <algorithm>
#include <random>
#include <ctime>

#include "trajectory.h"
#include "problemDisc2D.h"
#include "problemDisc1D.h"
#include "energy.h"
#include "tools.h"

/* -------------------------------- Parameter ------------------------------- */

int const ga::MAX_ITERATOR = 20; //50;
int const ga::POPULATION_SIZE = 20; //30;
double const ga::MUTATION_PROBABILITY = 0.005;

/* ------------------------------- Individual ------------------------------- */

ga::Individual::Individual(int heightDiscNum, int lengthDiscNum) : heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum) {
    // hmin = 0
    // hmax = heightDiscNum - 1
    trajectory = Trajectory(lengthDiscNum);
    for (int i = 0, h; i < lengthDiscNum; i++) {
        h = tools::randInt(0, heightDiscNum - 1);
        trajectory.setHeightIndex(i, h);
    }
}

ga::Individual::Individual(int heightDiscNum, int lengthDiscNum, Trajectory traj)
: heightDiscNum(heightDiscNum), lengthDiscNum(lengthDiscNum), trajectory(traj) {
    // empty
}

Trajectory ga::Individual::getTrajectory() const {
    return trajectory;
}

void ga::Individual::setCostINF() {
    cost = 1e7; // INF = 1e7
}

double ga::Individual::getCost() const {
    return cost;
}

void ga::Individual::calCost(const ProblemDisc2D &problem) {
    cost = calHeightCost() + calSpeedCost(problem);
}

double ga::Individual::calHeightCost() const {
    return trajectory.calHeightCost(resource::HEIGHT_COST_PROPOR);
}

double ga::Individual::calSpeedCost(const ProblemDisc2D &problem) const {
    return energy_calculator::calSpeedCost(problem, trajectory);
}

void ga::Individual::mutation() {
    double rd;
    for (int i = 0; i < lengthDiscNum; i++) {
        rd = tools::randDouble(0, 1);
        if (rd < ga::MUTATION_PROBABILITY) {
            int h = tools::randInt(0, heightDiscNum - 1); // hmin=0, hmax=heightDiscNum-1
            while (h == trajectory.getHeightIndex(i)) {
                h = tools::randInt(0, heightDiscNum - 1);
            }
            trajectory.setHeightIndex(i, h);
        }
    }
}

bool ga::Individual::operator< (const ga::Individual& _individual) const {
    return cost < _individual.cost;
    // return cost < _individual.getCost();
}

/* -------------------------------- GASovler -------------------------------- */

ga::GASolver::GASolver(ProblemDisc2D *prob) : problem(prob) {
    // TODO: complete
    heightDiscNum = problem->getHeightDiscNum();
    lengthDiscNum = problem->getLengthDiscNum();

    trajectory = Trajectory(lengthDiscNum);
}

double ga::GASolver::getCost() const {
    return cost;
}

Trajectory ga::GASolver::getTrajectory() const {
    return trajectory;
}

void ga::GASolver::solve() {
    Population parents;
    int bestIndex = -1;
    for (int i = 0; i < ga::POPULATION_SIZE; i++) {
        ga::Individual ind = ga::Individual(heightDiscNum, lengthDiscNum);
        while (!isFeasible(ind.getTrajectory())) {
            ind = ga::Individual(heightDiscNum, lengthDiscNum);
        }
        parents.push_back(ind); // ga::Individual(heightDiscNum, lengthDiscNum)
        parents[i].calCost(*problem);
        if (bestIndex == -1 || parents[i].getCost() < parents[bestIndex].getCost()) {
            bestIndex = i;
        }
    }

    trajectory = Trajectory(parents[bestIndex].getTrajectory());
    cost = parents[bestIndex].getCost();

    // 记录每次迭代后的optimal cost
    std::vector<double> optimalCostList;
    optimalCostList.push_back(cost);
    // optimalCostList.push_back(parents[bestIndex].getCost());

    // 用于给parents配对
    int indices[ga::POPULATION_SIZE];
    for (int i = 0; i < ga::POPULATION_SIZE; i++) indices[i] = i;

    int iter = 0;
    while (iter < ga::MAX_ITERATOR) {
        Population children;
        // todo 交叉
        std::shuffle(indices, indices + ga::POPULATION_SIZE, std::default_random_engine(std::time(NULL)));
        for (int i = 0; i < ga::POPULATION_SIZE; i += 2) {
            // parents[indices[i]] 和 parents[indices[i + 1]] 进行交叉
            crossover(children, parents[indices[i]], parents[indices[i + 1]]);
        }
        // todo 变异
        for (int i = 0; i < ga::POPULATION_SIZE; i++) {
            // parents[i].mutation();
            children[i].mutation();
            // if (!isFeasible(parents[i].getTrajectory())) parents[i].setCostINF();
            // else parents[i].calCost(*problem);
            if (!isFeasible(children[i].getTrajectory())) children[i].setCostINF();
            else children[i].calCost(*problem);
        }
        // todo 选择
        selection(parents, children);
        // todo 更新最优trajectory
        if (parents[0].getCost() < cost) {
            trajectory = Trajectory(parents[0].getTrajectory());
            cost = parents[0].getCost();
        }
        ++iter;

        // 记录optimal cost
        // optimalCostList.push_back(cost);
    }

    // 输出 optimalCostList 到文件
    // tools::printVector("GA", ".\\newnewexp\\exp_iter\\iter_results.txt", optimalCostList);
}

void ga::GASolver::crossover(ga::Population &children, ga::Individual p1, ga::Individual p2) const {
    Trajectory traj1 = p1.getTrajectory();
    Trajectory traj2 = p2.getTrajectory();
    // 交叉的区间
    int lpos = tools::randInt(0, lengthDiscNum - 1);
    int rpos = tools::randInt(0, lengthDiscNum - 1);
    if (lpos < rpos) std::swap(lpos, rpos);
    // 交换高度
    for (int i = lpos, temp; i <= rpos; i++) {
        temp = traj1.getHeightIndex(i);
        traj1.setHeightIndex(i, traj2.getHeightIndex(i));
        traj2.setHeightIndex(i, temp);
    }
    // 加入children中
    children.push_back(ga::Individual(heightDiscNum, lengthDiscNum, traj1));
    children.push_back(ga::Individual(heightDiscNum, lengthDiscNum, traj2));
}

void ga::GASolver::selection(ga::Population &parents, const ga::Population &children) const {
    parents.insert(parents.end(), children.begin(), children.end());
    sort(parents.begin(), parents.end());
    parents.erase(parents.begin() + ga::POPULATION_SIZE, parents.end());
}

bool ga::GASolver::isFeasible(Trajectory traj) const {
    int num = problem->getSensorNum();
    bool collected[num] = {false};
    for (int sid = 0; sid < num; sid++) {
        int lmost = lengthDiscNum - 1;
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
        if (!collected[sid]) {
            return false;
        }
    }
    return true;
}