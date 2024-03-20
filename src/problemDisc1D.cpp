#include "problemDisc1D.h"
using namespace resource;


void ProblemDisc1D::transformFromProblemDisc2D(const ProblemDisc2D &prob, const aco::Trajectory &traj) {
    sensorNum = prob.getSensorNum();
    length = prob.getLength();
    lengthDiscNum = prob.getLengthDiscNum();
    // 构造 sensorList
    sensorList.resize(sensorNum);
    for (int lenId = 0; lenId < lengthDiscNum; lenId++) {
        int hId = traj.getHeightIndex(lenId);
        for (int sId = 0; sId < sensorNum; sId++) {
            if (prob.getSensor(sId).isCovered(lenId, hId)) {
                sensorList[sId].coverList.push_back(lenId);
            }
        }
    }
    for (int sId = 0; sId < sensorNum; sId++) {
        sensorList[sId].time = prob.getSensor(sId).time;
        sensorList[sId].updateByCoverList();
    }
}

vector<SensorDisc> ProblemDisc1D::getSensorList() const {
    return sensorList;
}

SensorDisc ProblemDisc1D::getSensor(int index) const {
    return sensorList[index];
}

double ProblemDisc1D::getLength() const {
    return length;
}

int ProblemDisc1D::getLengthDiscNum() const {
    return lengthDiscNum;
}

int ProblemDisc1D::getSensorNum() const {
    return sensorNum;
}