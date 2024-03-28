#ifndef ACO_ONLINE_H
#define ACO_ONLINE_H

/* -------------------------------------------------------------------------- */
/*                            ACO (online version)                            */
/* -------------------------------------------------------------------------- */

#include "aco.h"

namespace online {

class ACOSolver_online {

private:
    ProblemDisc2D *problem;
    int sensorNum;
    int lengthIndexNum;
    int heightIndexNum;
    aco::Trajectory trajectory; // 路径
    vector<int> rBound;         // 辅助变量

public:
    

};



}



#endif