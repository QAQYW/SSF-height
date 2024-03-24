#ifndef NAIVE_H
#define NAIVE_H

#include "problemDisc2D.h"
#include "aco.h"

namespace naive_solver {



class NaiveSolver {
private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    aco::Trajectory trajectory;

public:
    NaiveSolver(ProblemDisc2D *prob);
    int getSensorNum() const;
    int getLengthIndexNum() const;
    int getHeightIndexNum() const;
    void solve();

private:
    void generateTrajectory(int trajLen);
};



}


#endif