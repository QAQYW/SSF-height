#include "dataGenerator.h"


/* ------------------------------ DataGenerator ----------------------------- */

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
    double maxXMult = 0.4 * length; //0.2
    double minXMult = MIN_X_MULT; //std::min(MIN_X_MULT, maxXMult / 2);//MIN_X_MULT;
    double maxYMult = maxHeight * 1.5; //1.3;// / 1.3;
    double minYMult = minHeight / 1.5; //1.3; // / 1.6; // heightList[0] / 1.35;
    for (int i = 0; i < sensorNum; i++) {
        // 数据传输时间
        // double time = tools::approx(tools::randDouble(MIN_TRANSMISSION_TIME, MAX_TRANSMISSION_TIME), resource::TIME_ULP);
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

        bool flag = false;
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
            // xDiff *= 3;
            if (xDiff <= 1.0) {
                flag = true;
                break;
            }
            double xLeft = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
            double xRight = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
            
            ++count;
            rangeStr = rangeStr + std::to_string(xLeft) + "\t" + std::to_string(xRight) + "\t\n";
        }

        if (flag) {
            i--;
            continue;
        }

        double temp3 = swell * swell + swell + 1;
        double temp4 = swell + 2 + 2 * std::sqrt(temp3);
        double temp5 = temp3 + swell + 1 + (swell + 2) * std::sqrt(temp3);
        double width = std::sqrt(temp4 * temp4 * temp4) / temp5 / xCoef;
        // // 数据传输时间 - 与width平方成正比
        // double time = tools::randDouble(MIN_TIME_RANGE_PROP, MAX_TIME_RANGE_PROP) * width * width;
        // 数据传输时间 - 与width成正比
        double time = tools::randDouble(MIN_TIME_RANGE_PROP, MAX_TIME_RANGE_PROP) * width * TIME_PROP;
        time = tools::approx(time, resource::TIME_ULP) / 10;
        
        // 保存传输时间
        fout << std::to_string(time) << "\n";
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
    double length = std::ceil(tools::randDouble(MIN_LENGTH_SENSOR_PROP, MAX_LENGTH_SENSOR_PROP) * sensorNum * 10);
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
    double maxXMult, minXMult, maxYMult, minYMult;
    // maxXMult = 0.4 * length;
    // minXMult = MIN_X_MULT;
    // maxYMult = maxHeight * 1.5;//1.35;
    // minYMult = minHeight / 1.5;//1.35; // heightList[0] / 1.35;
    // maxXMult = MAX_X_MULT_COEF * length;
    maxXMult = MAX_X_MULT;
    minXMult = MIN_X_MULT;
    maxYMult = MAX_Y_MULT;
    minYMult = MIN_Y_MULT;
    for (int i = 0; i < sensorNum; i++) {
        // 数据传输时间
        // double time = tools::approx(tools::randDouble(MIN_TRANSMISSION_TIME, MAX_TRANSMISSION_TIME), resource::TIME_ULP);
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

        // 传输范围
        bool flag = false;
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
            if (xDiff <= 1.0) {
                flag = true;
                break;
            }
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
        if (flag) {
            i--;
            continue;
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

        double temp3 = swell * swell + swell + 1;
        double temp4 = swell + 2 + 2 * std::sqrt(temp3);
        double temp5 = temp3 + swell + 1 + (swell + 2) * std::sqrt(temp3);
        double width = std::sqrt(temp4 * temp4 * temp4) / temp5 / xCoef;
        // // 数据传输时间 - 与width平方成正比
        // double time = tools::randDouble(MIN_TIME_RANGE_PROP, MAX_TIME_RANGE_PROP) * width * width;
        // 数据传输时间 - 与width成正比
        double time = tools::randDouble(MIN_TIME_RANGE_PROP, MAX_TIME_RANGE_PROP) * width * TIME_PROP;
        time = time * (maxYMult - minYMult) / (MAX_Y_MULT - MIN_Y_MULT);
        time = tools::approx(time, resource::TIME_ULP);

        // 保存传输时间
        fout << std::to_string(time) << "\n";
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

    // std::cout << "\nsave shape param to file:" << filename << "\n";

    // 路径长度
    fout << std::to_string(this->length) << "\n";
    // std::cout << std::to_string(this->length) << "\n";
    // 最小高度
    fout << std::to_string(this->MIN_HEIGHT) << "\n";
    // std::cout << std::to_string(this->MIN_HEIGHT) << "\n";
    // 最大高度
    fout << std::to_string(this->MAX_HEIGHT) << "\n";
    // std::cout << std::to_string(this->MAX_HEIGHT) << "\n";
    // 传感器数量
    fout << std::to_string(this->sensorNum) << "\n";
    // std::cout << std::to_string(this->sensorNum) << "\n";

    // 输出每个传感器的形状参数
    for (int sid = 0; sid < this->sensorNum; sid++) {
        for (int i = 0; i < 4; i++) {
            fout << std::to_string(shape[sid][i]) << "\t";
            // std::cout << std::to_string(shape[sid][i]) << "\t";
        }
        fout << "\n";
        // std::cout << "\n";
    }
    // std::cout << "\n";

    fout.close();

}

// DataGenerator::DataGenerator(std::string path, int num, double maxYMult, double maxXMultCoef, double maxTimeRangeProp, double maxSwell)
//  : savePath(path), sensorNum(num), MAX_Y_MULT(maxYMult), MAX_X_MULT_COEF(maxXMultCoef), MAX_TIME_RANGE_PROP(maxTimeRangeProp), MAX_SWELL(maxSwell) {
//     unit_height = resource::REF_UNIT_HEIGHT;
//     unit_length = resource::REF_UNIT_LENGTH;
//     // std::cout << "max_y_mult = " << std::to_string(maxYMult) << "\n";
// };

// DataGenerator::DataGenerator(std::string path, int num, double maxYMult, double maxXMultCoef, double timeProp, double maxSwell)
//  : savePath(path), sensorNum(num), MAX_Y_MULT(maxYMult), MAX_X_MULT_COEF(maxXMultCoef), TIME_PROP(timeProp), MAX_SWELL(maxSwell) {
//     unit_height = resource::REF_UNIT_HEIGHT;
//     unit_length = resource::REF_UNIT_LENGTH;
// }

DataGenerator::DataGenerator(std::string path, int num, double maxYMult, double maxXMult, double timeProp, double maxSwell)
 : savePath(path), sensorNum(num), MAX_Y_MULT(maxYMult), MAX_X_MULT(maxXMult), TIME_PROP(timeProp), MAX_SWELL(maxSwell) {
    unit_height = resource::REF_UNIT_HEIGHT;
    unit_length = resource::REF_UNIT_LENGTH;
}


/* ----------------------------- DataGenerator2 ----------------------------- */

DataGenerator2::DataGenerator2(std::string path, int sensor_num, double max_y_mult, double max_x_mult, double time_prop, double max_swell)
: savePath(path), sensorNum(sensor_num), MAX_Y_MULT(max_y_mult), MAX_X_MULT(max_x_mult), TIME_PROP(time_prop), MAX_SWELL(max_swell) {
    unitHeight = resource::REF_UNIT_HEIGHT;
    unitLength = resource::REF_UNIT_LENGTH;
}

void DataGenerator2::generate_save_online(unsigned int seed, int data_index) {

    std::ofstream fout;
    // 数据保存的路径
    std::string path = savePath + "\\" + filenameBaseOnline + std::to_string(data_index) + ".txt";
    fout.open(path);

    // 保存随机种子
    fout << std::to_string(seed) << "\n";
    
    srand(seed);

    // 等间隔 (unitHeight) 地生成离散的无人机飞行高度，存于heightList中
    std::vector<double> heightList;
    double uavh = MIN_UAV_HEIGHT;
    while (uavh <= MAX_UAV_HEIGHT) {
        heightList.push_back(uavh);
        uavh += unitHeight;
    }

    // 有多少个离散高度
    int heightDiscNum = heightList.size();
    // 高度最小值
    double minHeight = heightList[0];
    // 高度最大值
    double maxHeight = heightList.back();

    // 保存高度
    fout << std::to_string(heightDiscNum) << "\n";
    for (int i = 0; i < heightDiscNum; i++)
        fout << std::to_string(heightList[i]) << "\t";
    fout << "\n";

    // 总传感器数量为 this->sensorNum
    int subSensorNum[3];
    int avgNum = sensorNum / 3;
    int tempNum = sensorNum - avgNum * 3;
    for (int i = 0; i < 3; i++) {
        subSensorNum[i] = avgNum;
        if (i < tempNum) ++subSensorNum[i];
    }

    // 生成路径长度
    double length = 0;
    double subLength[3];
    for (int i = 0; i < 3; i++) {
        subLength[i] = std::ceil(tools::randDouble(MIN_LENGTH_SENSOR_PROP, MAX_LENGTH_SENSOR_PROP) * subSensorNum[i]);
        length += subLength[i];
    }
    this->length = length;

    // 保存路径长度和传感器数量
    fout << std::to_string(length) << "\n";
    fout << std::to_string(sensorNum) << "\n";

    /**
     * 记录形状参数，用于python程序的可视化
     * shape[i][0]: mid
     * shape[i][1]: xCoef
     * shape[i][2]: yCoef
     * shape[i][3]: swell
     */
    double shape[sensorNum][4];

    // 最大高度
    double deltaYMult = (MAX_Y_MULT - MIN_Y_MULT) / 3.0;
    double yMultSet[4];
    yMultSet[0] = MIN_Y_MULT;
    yMultSet[1] = MIN_Y_MULT + deltaYMult;
    yMultSet[2] = MIN_Y_MULT + 2 * deltaYMult;
    yMultSet[3] = MAX_Y_MULT;

    // std::vector<resource::SensorOnline2D> sensorList;
    int preLength = 0, shapeIndex = 0;
    for (int group = 0; group < 3; group++) {
        for (int i = 0; i < subSensorNum[group]; i++) {
            // 传感器横坐标（中心点）
            double mid = preLength + tools::randDouble(0, subLength[group]);
            // 膨胀系数
            double swell = tools::randDouble(MIN_SWELL, MAX_SWELL);
            // x 系数
            double xCoef = 1.0 / tools::randDouble(MIN_X_MULT, MAX_X_MULT);
            // y 系数
            double yCoef = 1.0 / tools::randDouble(yMultSet[group], yMultSet[group + 1]);
            // ? 最大高度
            double yMax = 2.0 / yCoef;
            if (yMax <= minHeight) {
                --i;
                continue;
            }

            // 传输范围
            bool flag = false;
            int count = 0;
            resource::SensorOnline2D sensor;
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
                if (xDiff <= 1.0) {
                    flag = true;
                    break;
                }
                resource::Range dataRange, controlRange;
                dataRange.left = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
                dataRange.right = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
                sensor.dataList.push_back(dataRange);
                
                // 控制通信范围
                xDiff *= DataGenerator2::CONTROL_RANGE_MULT;
                controlRange.left = tools::approx(std::max(0.0, mid - xDiff), resource::LENGTH_ULP);
                controlRange.right = tools::approx(std::min(length, mid + xDiff), resource::LENGTH_ULP);
                sensor.controlList.push_back(controlRange);
            }
            if (flag) {
                i--;
                continue;
            }
            resource::Range widest = resource::Range(sensor.dataList[0]);
            for (resource::Range rg : sensor.dataList) {
                widest.left = std::min(widest.left, rg.left);
                widest.right = std::max(widest.right, rg.right);
            }
            for (int j = 0; j < heightDiscNum; j++) {
                if (j < count) {
                    sensor.controlList[j].left = std::min(widest.left, sensor.controlList[j].left);
                    sensor.controlList[j].right = std::max(widest.right, sensor.controlList[j].right);
                } else {
                    resource::Range controlRange = resource::Range(widest);
                    sensor.controlList.push_back(controlRange);
                }
            }

            double temp3 = swell * swell + swell + 1;
            double temp4 = swell + 2 + 2 * std::sqrt(temp3);
            double temp5 = temp3 + swell + 1 + (swell + 2) * std::sqrt(temp3);
            double width = std::sqrt(temp4 * temp4 * temp4) / temp5 / xCoef;
            double time = tools::randDouble(MIN_TIME_RANGE_PROP, MAX_TIME_RANGE_PROP) * width * TIME_PROP;
            // time = time * (maxYMult - minYMult) / (MAX_Y_MULT - MIN_Y_MULT);
            time = tools::approx(time, resource::TIME_ULP);

            // 保存传输时间
            fout << std::to_string(time) << "\n";
            // 保存单个传感器的data range数量
            fout << std::to_string(count) << "\n";
            // 保存所有高度（若有）下的数据传输范围
            for (int j = 0; j < count; j++)
                fout << std::to_string(sensor.dataList[j].left) << "\t" << std::to_string(sensor.dataList[j].right) << "\t\n";
            // 保存所有高度下的控制通信范围（覆盖所有高度）
            for (int j = 0; j < heightDiscNum; j++)
                fout << std::to_string(sensor.controlList[j].left) << "\t" << std::to_string(sensor.controlList[j].right) << "\t\n";
            
            // 记录形状参数
            shape[shapeIndex][0] = mid;
            shape[shapeIndex][1] = xCoef;
            shape[shapeIndex][2] = yCoef;
            shape[shapeIndex][3] = swell;
            ++shapeIndex;
        }
        preLength += subLength[group];
    }

    fout.close();

    saveSensorShape(shape, data_index);
}

void DataGenerator2::saveSensorShape(double shape[][4], int data_index) const {
    std::ofstream fout;
    std::string path = savePath + "\\" + filenameBaseShape + std::to_string(data_index) + ".txt";
    fout.open(path);

    // 路径长度
    fout << std::to_string(length) << "\n";
    // 无人机最低高度
    fout << std::to_string(MIN_UAV_HEIGHT) << "\n";
    // 无人机最高高度
    fout << std::to_string(MAX_UAV_HEIGHT) << "\n";
    // 传感器数量
    fout << std::to_string(sensorNum) << "\n";

    // 每个传感器形状参数
    for (int i = 0; i < sensorNum; i++) {
        for (int j = 0; j < 4; j++) {
            fout << std::to_string(shape[i][j]) << "\t";
        }
        fout << "\n";
    }

    fout.close();
}