#include "ga.h"

#include "trajectory.h"
#include "problemDisc2D.h"
#include "problemDisc1D.h"
#include "energy.h"
#include "tools.h"


/* ------------------------------- Individual ------------------------------- */

ga::Individual::Individual() {
    // TODO: complete
}

Trajectory ga::Individual::getTrajectory() const {
    return trajectory;
}

double ga::Individual::getCost() const {
    return cost;
}

void ga::Individual::calCost(const ProblemDisc2D &problem) {
    cost = calHeightCost() + calSpeedCost(problem);
}

double ga::Individual::calHeightCost() const {
    return trajectory.calHeightCost();
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

/* -------------------------------- GASovler -------------------------------- */

ga::GASolver::GASolver(ProblemDisc2D *prob) : problem(prob) {
    // TODO: complete
}