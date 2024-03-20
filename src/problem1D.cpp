#include "problem1D.h"
using namespace resource;

// int Problem1D::getSensorNum() const {
//     return sensorNum;
// }

// double Problem1D::getLength() const {
//     return length;
// }

// vector<Sensor> Problem1D::getSensorList() const {
//     return sensorList;
// }

// Sensor Problem1D::getSensor(int index) const {
//     return sensorList[index];
// }

// void Problem1D::transformFromProblemDisc2D(const ProblemDisc2D &prob, const Trajectory &traj) {
//     sensorNum = prob.getSensorNum();
//     length = prob.getLength();
//     // 构造 sensorList
//     sensorList.resize(sensorNum);
//     vector<bool> visited(sensorNum, false);
//     int discNum = prob.getLengthDiscNum();
//     for (int lenId = 0; lenId < discNum; lenId++) {
//         int hId = traj.getHeightIndex(lenId);
//         for (int sId = 0; sId < sensorNum; sId++) {
//             if (prob.getSensor(sId).isCovered(lenId, hId)) {
//                 double len = resource::indexToLength(lenId);
//                 if (!visited[sId]) {
//                     sensorList[sId].range.left = len;
//                     visited[sId] = true;
//                 }
//                 sensorList[sId].range.right = len;
//             }
//         }
//     }
//     for (int sId = 0; sId < sensorNum; sId++) {
//         sensorList[sId].time = prob.getSensor(sId).time;
//     }
// }