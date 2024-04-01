#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

namespace tools {
    // 生成[minVal, maxVal]之间的double随机数
    double randDouble(double minVal, double maxVal);
    // 生成[minVal, maxVal]之间的int随机数
    int randInt(int minVal, int maxVal);
    // 按照所给精度 ulp 进行四舍五入，最终答案是 ulp 的倍数
    double approx(double val, const double ulp);
    /// @brief 以固定字符分割字符串
    /// @param strList 存分割结果
    /// @param str 原字符串
    /// @param ch 分隔符
    void splitString(std::vector<std::string> &strList, const std::string &str, char ch);
}

#endif