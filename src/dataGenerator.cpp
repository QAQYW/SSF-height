#include "dataGenerator.h"

void DataGenerator::generateAndSave(unsigned int seed, int dataIndex) {

    std::ofstream fout;
    std::string filename = this->savePath + "\\" + filenameBase + std::to_string(dataIndex) + ".txt";
    fout.open(filename);

    // 保存随机数种子
    fout << std::to_string(seed) << "\n";

    std::srand(seed);

    // 等间隔（unit_height）产生离散后的高度，存于heightList中
    std::vector<double> heightList;
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

    /*
        水滴型曲线参考
        https://zhuanlan.zhihu.com/p/380580061
    */

   /**
    * 保存用于python绘图的形状参数
    * shape[i][0]: mid
    * shape[i][1]: xCoef
    * shape[i][2]: yCoef
    * shape[i][3]: swell
   */
   double shape[sensorNum][4];

    // 随机产生每个传感器的传输范围
    // 几个辅助变量
    double maxXMult = 0.2 * length;
    double minXMult = std::min(MIN_X_MULT, maxXMult / 2);//MIN_X_MULT;
    double maxYMult = maxHeight * 1.3;// / 1.3;
    double minYMult = minHeight / 1.3; // / 1.6; // heightList[0] / 1.35;
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
        }

        // 保存传输时间
        fout << std::to_string(time) << "\n";

        int count = 0;
        std::string rangeStr = "";
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
            double xLeft = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
            double xRight = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
            
            ++count;
            rangeStr = rangeStr + std::to_string(xLeft) + "\t" + std::to_string(xRight) + "\t\n";
        }
        
        // 保存单个传感器range数量，以及各高度下（若有）的范围
        fout << std::to_string(count) << "\n";
        fout << rangeStr;
        
        // 记录形状参数
        shape[i][0] = mid;
        shape[i][1] = xCoef;
        shape[i][2] = yCoef;
        shape[i][3] = swell;
    }

    fout.close();

    saveSensorShape(shape, dataIndex);
}

void DataGenerator::generateAndSave_Online(unsigned int seed, int dataIndex) {

    std::ofstream fout;
    std::string filename = this->savePath + "\\online_" + filenameBase + std::to_string(dataIndex) + ".txt";
    fout.open(filename);

    // 保存随机数种子
    fout << std::to_string(seed) << "\n";

    srand(seed);

    // 等间隔（unit_height）产生离散后的高度，存于heightList中
    std::vector<double> heightList;
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

    /*
        水滴型曲线参考
        https://zhuanlan.zhihu.com/p/380580061
    */

   /**
    * 保存用于python绘图的形状参数
    * shape[i][0]: mid
    * shape[i][1]: xCoef
    * shape[i][2]: yCoef
    * shape[i][3]: swell
   */
   double shape[sensorNum][4];

    // 随机产生每个传感器的传输范围
    // 几个辅助变量
    double maxXMult = 0.4 * length;
    double minXMult = MIN_X_MULT;
    double maxYMult = maxHeight / 1.5;//1.35;
    double minYMult = minHeight / 1.5;//1.35; // heightList[0] / 1.35;
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
        }

        // 保存传输时间
        fout << std::to_string(time) << "\n";

        // 传输范围
        int count = 0;
        std::vector<double> dataLeft, dataRight;
        std::vector<double> controlLeft, controlRight;
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

            ++count;

            // 数据传输范围
            double xDiff = std::sqrt(temp2) / xCoef;
            double xLeft = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
            double xRight = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
            dataLeft.push_back(xLeft);
            dataRight.push_back(xRight);

            // 控制通信范围 (control communication range / control information delivery range)
            xDiff *= DataGenerator::CONTROL_RANGE_PROP;
            xLeft = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
            xRight = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
            controlLeft.push_back(xLeft);
            controlRight.push_back(xRight);
        }
        double lmost = dataLeft[0];
        double rmost = dataRight[0];
        for (double left : dataLeft)
            lmost = std::min(lmost, left);
        for (double right : dataRight)
            rmost = std::max(rmost, right);
        for (int j = 0; j < heightDiscNum; j++) {
            if (j < count) {
                controlLeft[j] = std::min(lmost, controlLeft[j]);
                controlRight[j] = std::max(rmost, controlRight[j]);
            } else {
                controlLeft.push_back(lmost);
                controlRight.push_back(rmost);
            }
        }
        // 保存单个传感器data range数量
        fout << std::to_string(count) << "\n";
        // 保存各高度下（若有）的数据传输范围
        for (int j = 0; j < count; j++)
            fout << std::to_string(dataLeft[j]) << "\t" << std::to_string(dataRight[j]) << "\t\n";
        // 保存各高度下的控制通信范围（覆盖所有高度）
        for (int j = 0; j < heightDiscNum; j++)
            fout << std::to_string(controlLeft[j]) << "\t" << std::to_string(controlRight[j]) << "\t\n";
        
        // 记录形状参数
        shape[i][0] = mid;
        shape[i][1] = xCoef;
        shape[i][2] = yCoef;
        shape[i][3] = swell;
    }

    fout.close();

    saveSensorShape(shape, dataIndex);
}

void DataGenerator::saveSensorShape(double shape[][4], int dataIndex) const {
    std::ofstream fout;
    std::string filename = this->savePath + "\\online_shape_" + std::to_string(dataIndex) + ".txt";
    fout.open(filename);

    std::cout << "\nsave shape param to file:" << filename << "\n";

    // 路径长度
    fout << std::to_string(this->length) << "\n";
    std::cout << std::to_string(this->length) << "\n";
    // 最小高度
    fout << std::to_string(this->MIN_HEIGHT) << "\n";
    std::cout << std::to_string(this->MIN_HEIGHT) << "\n";
    // 最大高度
    fout << std::to_string(this->MAX_HEIGHT) << "\n";
    std::cout << std::to_string(this->MAX_HEIGHT) << "\n";
    // 传感器数量
    fout << std::to_string(this->sensorNum) << "\n";
    std::cout << std::to_string(this->sensorNum) << "\n";

    // 输出每个传感器的形状参数
    for (int sid = 0; sid < this->sensorNum; sid++) {
        for (int i = 0; i < 4; i++) {
            fout << std::to_string(shape[sid][i]) << "\t";
            std::cout << std::to_string(shape[sid][i]) << "\t";
        }
        fout << "\n";
        std::cout << "\n";
    }
    std::cout << "\n";

    fout.close();

}