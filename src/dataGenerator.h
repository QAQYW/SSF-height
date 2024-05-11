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
    // ! 这个没用到 传感器数量最后都固定了
    const int MAX_SENSOR_NUM = 100;
    const int MIN_SENSOR_NUM = 2;

    // 传感器传输数据所需时间的上下界（秒）
    const double MAX_TRANSMISSION_TIME = 500; //10;
    const double MIN_TRANSMISSION_TIME = 10; //0.1;

    // 假设传输时间与传输范围的最大宽度相关的系数
    const double MAX_TIME_RANGE_PROP = 0.1; //1 / 5.0;
    const double MIN_TIME_RANGE_PROP = 2.0; //1 / 50.0; // 0.02
    // 假设传输时间与传输范围的最大宽度线性相关
    const double TIME_PROP = 1;

    // // 传感器传输范围的上下界
    // const double MAX_RANGE = 10; // ! 没想好取值
    // const double MIN_RANGE = 1;//0.5; // ! 0.5米会不会太小

    // 飞行高度范围
    const double MAX_HEIGHT = 120; //200; // 单位：米
    // const double MAX_HEIGHT = 140; // for tiny_test only
    const double MIN_HEIGHT = 80; // 单位：米

    // 路径长度与传感器数量的比例系数
    const double MAX_LENGTH_SENSOR_PROP = 50; //15; //20; // 100;
    // const double MAX_LENGTH_SENSOR_PROP = 3; // for tiny_test only
    const double MIN_LENGTH_SENSOR_PROP = 20; // 10;

    
    // 传感器范围x坐标最小放大倍数（仅用于生成传输范围吗）
    const double MIN_X_MULT = 20; //10;
    const double MAX_X_MULT = 50;
    const double MAX_X_MULT_COEF = 0.2; // ! 弃用，让传输范围与路径总长度完全独立

    const double MIN_Y_MULT = 60; //105;
    const double MAX_Y_MULT = 115;

    /**
     *  范围膨胀系数
     * 越小越接近圆形，等于 0 时退化为圆形
     * 参考 https://zhuanlan.zhihu.com/p/380580061
    */
    /// @brief 范围膨胀系数最小值
    const double MIN_SWELL = 0;
    /// @brief 范围膨胀系数最大值
    const double MAX_SWELL = 3; //2;

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
    /// @param path 输出路径
    /// @param num 传感器数量
    DataGenerator(std::string path, int num): savePath(path), sensorNum(num), unit_height(resource::REF_UNIT_HEIGHT), unit_length(resource::REF_UNIT_LENGTH) {};

    // /// @brief 构造函数
    // /// @param path 
    // /// @param num 
    // /// @param maxYMult 
    // /// @param maxXMultCoef 
    // /// @param maxTimeRangeProp 
    // /// @param maxSwell 
    // DataGenerator(std::string path, int num, double maxYMult, double maxXMultCoef, double maxTimeRangeProp, double maxSwell);
    // //: savePath(path), sensorNum(num), MAX_Y_MULT(maxYMult), MIN_Y_MULT(105), MAX_X_MULT_COEF(maxXMultCoef), MAX_TIME_RANGE_PROP(maxTimeRangeProp), MAX_SWELL(maxSwell) {};
    
    // /// @brief 构造函数
    // /// @param path 
    // /// @param num 
    // /// @param maxYMult 
    // /// @param maxXMultCoef 
    // /// @param timeProp 
    // /// @param maxSwell 
    // DataGenerator(std::string path, int num, double maxYMult, double maxXMultCoef, double timeProp, double maxSwell);

    /// @brief 构造函数
    /// @param path 
    /// @param num 
    /// @param maxYMult 
    /// @param maxXMult 
    /// @param timeProp 
    /// @param maxSwell 
    DataGenerator(std::string path, int num, double maxYMult, double maxXMult, double timeProp, double maxSwell);
    
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


class DataGenerator2 {

private:
    std::string savePath;

public:
    // const std::string filenameBaseOffline = "test_data_";
    const std::string filenameBaseOnline = "online_test_data_";
    const std::string filenameBaseShape = "online_shape_";

    // 传输时间与水滴宽度的比例系数 增大的倍数
    const double TIME_PROP = 1;

    // 传输时间与水滴宽度的比例系数
    const double MIN_TIME_RANGE_PROP = 0.1;
    const double MAX_TIME_RANGE_PROP = 2.0;
    
    // 无人机飞行高度
    const double MIN_UAV_HEIGHT = 80;
    const double MAX_UAV_HEIGHT = 120;

    // 路径长度与传感器数量的比值
    const double MIN_LENGTH_SENSOR_PROP = 20;
    const double MAX_LENGTH_SENSOR_PROP = 50;

    // 传输范围宽度系数
    const double MIN_X_MULT = 20;
    const double MAX_X_MULT = 50;

    // 传输范围高度系数
    const double MIN_Y_MULT = 40;
    const double MAX_Y_MULT = 100;

    // 水滴曲线的膨胀系数
    const double MIN_SWELL = 0;
    const double MAX_SWELL = 3;

    // control communication range与data transmission range的半径比
    const double CONTROL_RANGE_MULT = 1.5;

private:
    double unitHeight;
    double unitLength;
    int sensorNum;
    double length;

public:
    /// @brief 构造函数
    /// @param path 
    /// @param sensor_num 
    /// @param max_y_mult 
    /// @param max_x_mult 
    /// @param time_prop 
    /// @param max_swell 
    DataGenerator2(std::string path, int sensor_num, double max_y_mult, double max_x_mult, double time_prop, double max_swell);

    void generate_save_online(unsigned int seed, int data_index);
    void saveSensorShape(double shape[][4], int data_index) const;
};



#endif