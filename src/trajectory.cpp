#include "trajectory.h"

Trajectory::Trajectory(int size) {
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

vector<int> Trajectory::getHeightSche() const {
    return heightSche;
}

int Trajectory::getHeightIndex(int lengthIndex) const {
    return heightSche[lengthIndex];
}

double Trajectory::calHeightCost() const {
    int size = heightSche.size();
    double cost = 0;
    for (int i = 1; i < size; i++) {
        cost += resource::costByHeight(heightSche[i - 1], heightSche[i]);
    }
    return cost;
}

double Trajectory::calSpeedCost(const ProblemDisc2D &problem2D) const {
    // double cost = 0;
    ProblemDisc1D problem1D;
    problem1D.transformFromProblemDisc2D(problem2D, *this); // ????????????
    ssf::SSFSolverDisc ssfSolver(&problem1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    return ssfSolver.getCost();
}

double Trajectory::calSpeedCost(const ProblemDisc2D &problem2D, vector<double> &speedSche) const {
    // double cost = 0;
    ProblemDisc1D problem1D;
    problem1D.transformFromProblemDisc2D(problem2D, *this); // ????????????
    ssf::SSFSolverDisc ssfSolver(&problem1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    speedSche = ssfSolver.getSolution().getSpeedSche();
    return ssfSolver.getCost();
}