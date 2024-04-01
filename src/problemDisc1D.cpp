#include "problemDisc1D.h"

// void ProblemDisc1D::transformFromProblemDisc2D(const ProblemDisc2D &prob, const Trajectory &traj) {
//     sensorNum = prob.getSensorNum();
//     length = prob.getLength();
//     lengthDiscNum = prob.getLengthDiscNum();
//     // 构造 sensorList
//     sensorList.resize(sensorNum);
//     for (int lenid = 0; lenid < lengthDiscNum; lenid++) {
//         int hid = traj.getHeightIndex(lenid);
//         for (int sid = 0; sid < sensorNum; sid++) {
//             if (prob.getSensor(sid).isCovered(lenid, hid)) {
//                 sensorList[sid].coverList.push_back(lenid);
//             }
//         }
//     }
//     for (int sid = 0; sid < sensorNum; sid++) {
//         sensorList[sid].time = prob.getSensor(sid).time;
//         sensorList[sid].updateByCoverList();
//     }
// }

ProblemDisc1D::ProblemDisc1D(int num, double len, int lenNum, const std::vector<resource::SensorDisc2D> &list, const Trajectory &traj)
 : sensorNum(num), length(len), lengthDiscNum(lenNum) {
    // 构造 sensorList
    sensorList.resize(sensorNum);
    for (int lenid = 0; lenid < lengthDiscNum; lenid++) {
        int hid = traj.getHeightIndex(lenid);
        for (int sid = 0; sid < sensorNum; sid++) {
            if (list[sid].isCovered(lenid, hid)) {
                sensorList[sid].coverList.push_back(lenid);
            }
        }
    }
    for (int sid = 0; sid < sensorNum; sid++) {
        sensorList[sid].time = list[sid].time;
        sensorList[sid].updateByCoverList();
    }
}

std::vector<resource::SensorDisc> ProblemDisc1D::getSensorList() const {
    return sensorList;
}

resource::SensorDisc ProblemDisc1D::getSensor(int index) const {
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
