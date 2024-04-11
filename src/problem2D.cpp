#include "problem2D.h"

int Problem2D::getSensorNum() const {
    return sensorNum;
}

double Problem2D::getLength() const {
    return length;
}

std::vector<resource::Sensor2D> Problem2D::getSensorList() const {
    return sensorList;
}

resource::Sensor2D Problem2D::getSensor(int index) const {
    return sensorList[index];
}

double Problem2D::getMinHeight() const {
    return minHeight;
}

double Problem2D::getMaxHeight() const {
    return maxHeight;
}

int Problem2D::getHeightDiscNum() const {
    return heightDiscNum;
}

std::vector<double> Problem2D::getHeightList() const {
    return heightList;
}

void Problem2D::initFromFile(const std::string &filename) {
    std::ifstream fin;
    fin.open(filename);
    std::string buff;

    // 随机数种子
    std::getline(fin, buff);
    this->seed = std::stoi(buff);

    // 高度离散的数量
    std::getline(fin, buff);
    this->heightDiscNum = std::stoi(buff);

    // 各离散高度值
    std::getline(fin, buff);
    std::vector<std::string> heightStrs;
    tools::splitString(heightStrs, buff, '\t');
    for (int i = 0; i < this->heightDiscNum; i++) {
        this->heightList.push_back(std::stod(heightStrs[i]));
    }
    this->minHeight = this->heightList[0];
    this->maxHeight = this->heightList.back();

    // 路径长度
    std::getline(fin, buff);
    this->length = std::stod(buff);

    // 传感器数量
    std::getline(fin, buff);
    this->sensorNum = std::stoi(buff);
    
    // 各传感器信息
    this->sensorList.resize(this->sensorNum);
    for (int i = 0; i < this->sensorNum; i++) {
        // 传输时间
        std::getline(fin, buff);
        sensorList[i].time = std::stod(buff);

        // range跨过的高度数量
        std::getline(fin, buff);
        int num = std::stoi(buff);

        // 各高度下的range
        sensorList[i].rangeList.resize(num);
        for (int j = 0; j < num; j++) {
            std::getline(fin, buff);
            std::vector<std::string> rangeStr;
            tools::splitString(rangeStr, buff, '\t');
            sensorList[i].rangeList[j].left = std::stod(rangeStr[0]);
            sensorList[i].rangeList[j].right = std::stod(rangeStr[1]);
        }
    }

    fin.close();
}

void Problem2D::initFromOnlineFile(const std::string &filename) {
    std::ifstream fin;
    fin.open(filename);
    std::string buff;

    // 随机数种子
    std::getline(fin, buff);
    this->seed = std::stoi(buff);

    // 高度离散的数量
    std::getline(fin, buff);
    this->heightDiscNum = std::stoi(buff);

    // 各离散高度值
    std::getline(fin, buff);
    std::vector<std::string> heightStrs;
    tools::splitString(heightStrs, buff, '\t');
    for (int i = 0; i < this->heightDiscNum; i++) {
        this->heightList.push_back(std::stod(heightStrs[i]));
    }
    this->minHeight = this->heightList[0];
    this->maxHeight = this->heightList.back();

    // 路径长度
    std::getline(fin, buff);
    this->length = std::stod(buff);

    // 传感器数量
    std::getline(fin, buff);
    this->sensorNum = std::stoi(buff);

    // 各传感器信息
    this->sensorList.resize(this->sensorNum);
    for (int i = 0; i < this->sensorNum; i++) {
        // 传输时间
        std::getline(fin, buff);
        sensorList[i].time = std::stod(buff);

        // data range 跨过的高度
        std::getline(fin, buff);
        int num = std::stoi(buff);

        std::vector<std::string> rangeStr;
        // 各高度下data range
        sensorList[i].rangeList.resize(num);
        for (int j = 0; j < num; j++) {
            std::getline(fin, buff);
            rangeStr.clear();
            tools::splitString(rangeStr, buff, '\t');
            sensorList[i].rangeList[j].left = std::stod(rangeStr[0]);
            sensorList[i].rangeList[j].right = std::stod(rangeStr[1]);
        }
        // 各高度下的control range
        // 忽略这部分数据，读入但不计算或保存
        // sensorList[i].controlList.resize(this->heightDiscNum);
        for (int j = 0; j < this->heightDiscNum; j++) {
            std::getline(fin, buff);
            // tools::splitString(rangeStr, buff, '\t');
            // sensorList[i].controlList[j].left = std::stod(rangeStr[0]);
            // sensorList[i].controlList[j].right = std::stod(rangeStr[1]);
        }
    }

    fin.close();
}