#include "energy.h"

double energy_calculator::calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj) {
    // ProblemDisc1D prob1D;
    // prob1D.transformFromProblemDisc2D(prob2D, traj);
    ProblemDisc1D prob1D = ProblemDisc1D(prob2D.getSensorNum(), prob2D.getLength(), prob2D.getLengthDiscNum(), prob2D.getSensorList(), traj);
    ssf::SSFSolverDisc ssfSolver(&prob1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    return ssfSolver.getCost();
}

double energy_calculator::calSpeedCost(const ProblemDisc2D &prob2D, const Trajectory &traj, std::vector<double> &speedSche) {
    // ProblemDisc1D prob1D;
    // prob1D.transformFromProblemDisc2D(prob2D, traj);
    ProblemDisc1D prob1D = ProblemDisc1D(prob2D.getSensorNum(), prob2D.getLength(), prob2D.getLengthDiscNum(), prob2D.getSensorList(), traj);
    ssf::SSFSolverDisc ssfSolver(&prob1D);
    ssfSolver.solve();
    ssfSolver.calCost();
    speedSche = ssfSolver.getSolution().getSpeedSche();
    return ssfSolver.getCost();
}
