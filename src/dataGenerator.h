#ifndef DATA_GENERATOR_H
#define DATA_GENERATOR_H

#include <string>
#include <fstream>
#include <string>

#include "tools.h"
#include "resource.h"

// ? the upper bound and lower bound of parameters
// ? are defined in namespace resource?

class DataGenerator {
private:
    // 文件输出
    string savePath; // 数据输出路径

public:
    const string filenameBase = "test_data_";
    // 传感器数量上下界
    const int MAX_SENSOR_NUM = 100;
    const int MIN_SENSOR_NUM = 2;
    // 传感器传输数据所需时间的上下界
    const double MAX_TRANSMISSION_TIME = 60; //10;    // ! 单位是秒还是分钟
    const double MIN_TRANSMISSION_TIME = 1; //0.1;   // ! 单位是秒还是分钟
    // 传感器传输范围的上下界
    const double MAX_RANGE = 10; // ! 没想好取值
    const double MIN_RANGE = 1;//0.5; // ! 0.5米？会不会太小
    // 飞行高度范围
    const double MAX_HEIGHT = 200; // 单位：米
    const double MIN_HEIGHT = 100; // 单位：米
    // 路径长度与传感器数量的比例系数
    const double MAX_LENGTH_SENSOR_PROP = 50; // 10;
    const double MIN_LENGTH_SENSOR_PROP = 20; // 1;
    // ? 下面是不太清楚缘由的一部分，知道怎么用，不知道从何而来
    // 传感器范围x坐标最小放大倍数（仅用于生成传输范围吗）
    const double MIN_X_MULT = 10;
    // 范围膨胀系数
    const double MIN_SWELL = 0; // ? 可以为0吗
    const double MAX_SWELL = 2;
    // TODO 暂缺

private:
    double unit_height; // 高度离散化的最小粒度
    double unit_length; // 距离离散化的最小粒度
    int sensorNum;      // 传感器数量
    double length;      // 路径长度 // ! 没用到

public:
    // 构造函数（若不指定sensorNum，则默认为0，后续随机生成）
    DataGenerator(string path): savePath(path), sensorNum(0), unit_height(resource::REF_UNIT_HEIGHT), unit_length(0.1) {};
    DataGenerator(string path, int num): savePath(path), sensorNum(num), unit_height(10), unit_length(0.1) {};
    // 生成数据，并保存到文件
    void generateAndSave(unsigned int seed, int dataIndex);
};


#endif