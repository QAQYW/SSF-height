#include "dataGenerator.h"

// ? seed的作用与含义是什么
void DataGenerator::generateAndSave(unsigned int seed, int dataIndex) {
    // ? 是否要分成多个文件存储不同的信息
    // ? 要不要存成 json

    ofstream fout;
    string filename = this->savePath + "\\" + filenameBase + std::to_string(dataIndex) + ".txt";
    fout.open(filename);

    // 保存随机数种子
    fout << std::to_string(seed) << "\n";

    srand(seed);

    // 等间隔（unit_height）产生离散后的高度，存于heightList中
    vector<double> heightList;
    double h = MIN_HEIGHT;
    while (h <= MAX_HEIGHT) {
        heightList.push_back(h);
        h += unit_height;
    }
    // 离散高度值的数量
    int heightDiscNum = heightList.size();
    // 高度最小值
    double minHeight = MIN_HEIGHT;
    // 高度最大值
    double maxHeight = heightList.back();

    // 保存离散的高度数量，以及各高度值
    fout << std::to_string(heightDiscNum) << "\n";
    for (int i = 0; i < heightDiscNum; i++)
        fout << std::to_string(heightList[i]) << "\t";
    fout << "\n";

    // 产生传感器数量，并根据它产生路径长度
    int sensorNum;// = this->sensorNum == 0 ? tools::randInt(MIN_SENSOR_NUM, MAX_SENSOR_NUM) : this->sensorNum;
    if (this->sensorNum == 0) {
        sensorNum = tools::randInt(MIN_SENSOR_NUM, MAX_SENSOR_NUM);
        this->sensorNum = sensorNum;
    } else {
        sensorNum = this->sensorNum;
    }
    double length = std::ceil(tools::randDouble(MIN_LENGTH_SENSOR_PROP, MAX_LENGTH_SENSOR_PROP) * sensorNum);
    this->length = length;

    // 保存路径长度，传感器数量，以及各传感器信息
    fout << std::to_string(length) << "\n";
    fout << std::to_string(sensorNum) << "\n";

    // 随机产生每个传感器的传输范围
    // 几个辅助变量
    double maxXMult = 0.8 * length;
    double minXMult = MIN_X_MULT;
    double maxYMult = maxHeight / 1.35;
    double minYMult = minHeight / 1.35; // heightList[0] / 1.35;
    for (int i = 0; i < sensorNum; i++) {
        // 数据传输时间
        double time = tools::approx(tools::randDouble(MIN_TRANSMISSION_TIME, MAX_TRANSMISSION_TIME), resource::TIME_ULP);
        // 范围中心点横坐标
        double mid = tools::randDouble(0, length);
        // 范围膨胀系数
        double swell = tools::randDouble(MIN_SWELL, MAX_SWELL);
        // x 系数
        double xCoef = 1.0 / tools::randDouble(minXMult, maxXMult);
        // y 系数
        double yCoef = 1.0 / tools::randDouble(minYMult, maxYMult);
        double yMax = 2.0 / yCoef;
        if (yMax <= minHeight) {
            i--;
            continue;
            // ? 是要重新生成？
        }

        // 保存传输时间
        fout << std::to_string(time) << "\n";

        int count = 0;
        string rangeStr = "";
        for (int j = 0; j < heightDiscNum; j++) {
            double y = heightList[j];
            if (y >= yMax) break;

            double yBar = yCoef * y;
            double p = 2 * yBar * (yBar - 1) + swell;
            double q = yBar * yBar * yBar * (yBar - 2);

            double temp1 = p * p - 4 * q;
            if (temp1 <= 0) break;

            double temp2 = (std::sqrt(temp1) - p) / 2;
            if (temp2 <= 0) break;

            double xDiff = std::sqrt(temp2) / xCoef;
            double xLeft = tools::approx(max(0.0, mid - xDiff), resource::LENGTH_ULP);
            double xRight = tools::approx(min(length, mid + xDiff), resource::LENGTH_ULP);
            // ? data transmission range 有可能不是对称的？？
            
            ++count;
            rangeStr = rangeStr + std::to_string(xLeft) + "\t" + std::to_string(xRight) + "\t\n";
        }
        
        // 保存单个传感器range数量，以及各高度下（若有）的范围
        fout << std::to_string(count) << "\n";
        fout << rangeStr;
    }

    fout.close();
}