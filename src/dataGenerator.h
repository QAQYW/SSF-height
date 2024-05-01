#ifndef DATA_GENERATOR_H
#define DATA_GENERATOR_H

#include <string>
#include <iostream>
#include <fstream>

#include "tools.h"
#include "resource.h"

class DataGenerator {

private:
    /// @brief 保存路径（输出问题数据）
    std::string savePath;
public:
    const std::string filenameBase = "test_data_";

    // 传感器数量上下界
    const int MAX_SENSOR_NUM = 100;
    const int MIN_SENSOR_NUM = 2;

    // 传感器传输数据所需时间的上下界
    const double MAX_TRANSMISSION_TIME = 500; //10;    // ! 单位是秒还是分钟
    const double MIN_TRANSMISSION_TIME = 10; //0.1;   // ! 单位是秒还是分钟

    // 传感器传输范围的上下界
    const double MAX_RANGE = 10; // ! 没想好取值
    const double MIN_RANGE = 1;//0.5; // ! 0.5米会不会太小

    // 飞行高度范围
    const double MAX_HEIGHT = 200; // 单位：米
    // const double MAX_HEIGHT = 140; // for tiny_test only
    const double MIN_HEIGHT = 100; // 单位：米

    // 路径长度与传感器数量的比例系数
    const double MAX_LENGTH_SENSOR_PROP = 10; //15; //20; // 100;
    // const double MAX_LENGTH_SENSOR_PROP = 3; // for tiny_test only
    const double MIN_LENGTH_SENSOR_PROP = 5; // 10;

    // ? 下面是不太清楚缘由的一部分，知道怎么用，不知道从何而来
    // 传感器范围x坐标最小放大倍数（仅用于生成传输范围吗）
    const double MIN_X_MULT = 5; //10;

    /**
     *  范围膨胀系数
     * 越小越接近圆形，等于 0 时退化为圆形
     * 参考 https://zhuanlan.zhihu.com/p/380580061
    */
    /// @brief 范围膨胀系数最小值
    const double MIN_SWELL = 0;
    /// @brief 范围膨胀系数最大值
    const double MAX_SWELL = 1; //2;

    // control range与data transmission range的半径比
    const double CONTROL_RANGE_PROP = 1.5;

private:
    double unit_height; // 高度离散化的单位高度
    double unit_length; // 距离离散化的单位长度
    int sensorNum;      // 传感器数量
    double length;      // 路径长度

public:
    /// @brief 构造函数，不指定传感器数量，默认为0，后续会随机生成
    /// @param path 输出路径
    DataGenerator(std::string path, double uh, double ul): savePath(path), sensorNum(0), unit_height(resource::REF_UNIT_HEIGHT), unit_length(resource::REF_UNIT_LENGTH) {};
    
    /// @brief 构造函数，指定传感器数量，非随机生成
    /// @param path 随机种子
    /// @param num 传感器数量
    DataGenerator(std::string path, int num): savePath(path), sensorNum(num), unit_height(resource::REF_UNIT_HEIGHT), unit_length(resource::REF_UNIT_LENGTH) {};
    
    /// @brief 生成离线问题数据，并保存到文件
    /// @param seed 随机种子
    /// @param dataIndex 样例数据序号
    void generateAndSave(unsigned int seed, int dataIndex);
    
    /// @brief 生成在线问题数据，并保存到文件
    /// @param seed 随机种子
    /// @param dataIndex 样例数据序号
    void generateAndSave_Online(unsigned int seed, int dataIndex);
    /// @brief 保存所有传感器的数据传输范围的形状参数，方便用python进行可视化
    /// @param shape 形状参数
    /// @param dataIndex 样例序号
    void saveSensorShape(double shape[][4], int dataIndex) const;
};

#endif