#include "problemOnline2D.h"

int ProblemOnline2D::getSensorNum() const {
    return sensorNum;
}

double ProblemOnline2D::getLength() const {
    return length;
}

vector<resource::SensorOnline2D> ProblemOnline2D::getSensorList() const {
    return sensorList;
}

resource::SensorOnline2D ProblemOnline2D::getSensor(int index) const {
    return sensorList[index];
}

double ProblemOnline2D::getMinHeight() const {
    return minHeight;
}

double ProblemOnline2D::getMaxHeight() const {
    return maxHeight;
}

int ProblemOnline2D::getHeightDiscNum() const {
    return heightDiscNum;
}

// range的每一行，先data transmission range，再control communication range
void ProblemOnline2D::initFromFile(const string &filename) {
    ifstream fin;
    fin.open(filename);
    string buff;

    // 随机数种子
    getline(fin, buff);
    this->seed = std::stoi(buff);

    // 高度离散的数量
    getline(fin, buff);
    this->heightDiscNum = std::stoi(buff);

    // 各离散高度值
    getline(fin, buff);
    vector<string> heightStrs;
    tools::splitString(heightStrs, buff, '\t');
    for (int i = 0; i < this->heightDiscNum; i++) {
        this->heightList.push_back(std::stod(heightStrs[i]));
    }
    this->minHeight = this->heightList[0];
    this->maxHeight = this->heightList.back();

    // 路径长度
    getline(fin, buff);
    this->length = std::stod(buff);

    // 传感器数量
    getline(fin, buff);
    this->sensorNum = std::stoi(buff);

    // 各传感器信息
    this->sensorList.resize(this->sensorNum);
    for (int i = 0; i < this->sensorNum; i++) {
        // 传输时间
        getline(fin, buff);
        sensorList[i].time = std::stod(buff);

        // range跨过的高度数量
        getline(fin, buff);
        int num = std::stoi(buff);

        // 各高度下的range
        sensorList[i].dataList.resize(num);
        sensorList[i].controlList.resize(num);
        for (int j = 0; j < num; j++) {
            getline(fin, buff);
            vector<string> rangeStr;
            tools::splitString(rangeStr, buff, '\t');
            // data transmission range
            sensorList[i].dataList[j].left = std::stod(rangeStr[0]);
            sensorList[i].dataList[j].right = std::stod(rangeStr[1]);
            // control communication range
            sensorList[i].controlList[j].left = std::stod(rangeStr[2]);
            sensorList[i].controlList[j].right = std::stod(rangeStr[3]);
        }
    }

    fin.close();
}