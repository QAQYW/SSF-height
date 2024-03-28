#include "acoOnline.h"

/* --------------------------------- Sensor --------------------------------- */


/* ---------------------------- ACOSolver_Online ---------------------------- */

online::ACOSolver_Online::ACOSolver_Online(ProblemOnlineDisc2D *prob): problem(prob), trajectory() {
    sensorNum = prob->getSensorNum();
    lengthIndexNum = prob->getLengthDiscNum();
    heightIndexNum = prob->getHeightDiscNum();

    rBound.resize(lengthIndexNum, 0);
    // TODO 这部分得等进入了control range才能更新

    sensors.clear();
    for (int i = 0; i < sensorNum; i++) {
        // sensors.emplace_back(online::Sensor());
    }
}

ProblemOnlineDisc2D* online::ACOSolver_Online::getProblem() const {
    return problem;
}

aco::Trajectory online::ACOSolver_Online::getTrajectory() const {
    return trajectory;
}

void online::ACOSolver_Online::solve() {
    int countInformed = 0;
    vector<bool> informed(sensorNum, false);
    vector<bool> visited(sensorNum, false);

    int hMin = problem->getMinHeightIndex();
    int hMax = problem->getMaxHeightIndex();
    vector<resource::SensorOnlineDisc2D> sensors = problem->getSensorList();

    // 刚开始可以获得所有control range覆盖了d=0的info of sensor
    // 满足 min{sensorList[i].controlList[j] == 0} 即可
    int tempEnd = 0;
    for (int s = 0; s < sensorNum; s++) {
        for (int h = hMin; h <= hMax; h++) {
            if (sensors[s].controlList[h].leftIndex == 0) {
                informed[s] = true;
                ++countInformed;
                break;
            }
        }
        if (informed[s]) {
            for (int h = hMin; h <= hMax; h++) {
                tempEnd = max(tempEnd, sensors[s].dataList[h].rightIndex);
            }
        }
    }

    int trajLen = lengthIndexNum;
    trajectory = aco::Trajectory(trajLen, hMin);
    for (int d = 0; d < trajLen; d++) {
        // TODO try to get new info
        // ...
        // TODO reschedule the trajectory
        resolve(d, tempEnd);

        // TODO choose the height of next position
        int next;
        // TODO switch a sensor to link
        // ! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
}