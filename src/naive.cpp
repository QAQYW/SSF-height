#include "naive.h"


naive_solver::NaiveSolver::NaiveSolver(ProblemDisc2D *prob)
 : problem(prob) {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();
}

int naive_solver::NaiveSolver::getSensorNum() const {
    return sensorNum;
}

int naive_solver::NaiveSolver::getLengthIndexNum() const {
    return lengthIndexNum;
}

int naive_solver::NaiveSolver::getHeightIndexNum() const {
    return heightIndexNum;
}

void naive_solver::NaiveSolver::solve() {
    //
}

void naive_solver::NaiveSolver::generateTrajectory(int trajLen) {
    trajectory = aco::Trajectory(trajLen);
}