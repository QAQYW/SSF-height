#include "trajectory.h"

Trajectory::Trajectory(int size) {
    // 最低飞行高度的编号为0
    heightSche.resize(size, 0);
}

Trajectory::Trajectory(int size, int heightIndex) {
    heightSche.resize(size, heightIndex);
}

void Trajectory::reInit(int size, int heightIndex) {
    heightSche.resize(size, heightIndex);
}

void Trajectory::setHeightIndex(int lengthIndex, int heightIndex) {
    heightSche[lengthIndex] = heightIndex;
}

void Trajectory::addList(int heightIndex) {
    heightSche.push_back(heightIndex);
}

std::vector<int> Trajectory::getHeightSche() const {
    return heightSche;
}

int Trajectory::getHeightIndex(int lengthIndex) const {
    return heightSche[lengthIndex];
}

double Trajectory::calHeightCost(double coef) const {
    int size = heightSche.size();
    double cost = 0;
    for (int i = 1; i < size; i++) {
        double dh = std::abs(heightSche[i - 1] - heightSche[i]) * resource::REF_UNIT_HEIGHT;
        cost += resource::costByHeight(dh, coef);
    }
    return cost;
}

// // TODO ProblemDisc2D

// double Trajectory::calSpeedCost(const ProblemDisc2D &problem2D) const {
//     ProblemDisc1D problem1D;
//     problem1D.transformFromProblemDisc2D(problem2D, *this);
//     ssf::SSFSolverDisc ssfSolver(&problem1D);
//     ssfSolver.solve();
//     ssfSolver.calCost();
//     return ssfSolver.getCost();
//     return 0;
// }

// // TODO ProblemDisc2D

// double Trajectory::calSpeedCost(const ProblemDisc2D &problem2D, std::vector<double> &speedSche) const {
//     ProblemDisc1D problem1D;
//     problem1D.transformFromProblemDisc2D(problem2D, *this);
//     ssf::SSFSolverDisc ssfSolver(&problem1D);
//     ssfSolver.solve();
//     ssfSolver.calCost();
//     speedSche = ssfSolver.getSolution().getSpeedSche();
//     return ssfSolver.getCost();
//     return 0;
// }