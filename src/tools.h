#ifndef TOOL_H
#define TOOL_H

#include <vector>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <string>

using namespace std;

namespace tools {
    // 生成[minVal, maxVal]之间的double随机数
    double randDouble(double minVal, double maxVal);
    // 生成[minVal, maxVal]之间的int随机数
    int randInt(int minVal, int maxVal);
    // 按照所给精度 ulp 进行四舍五入，最终答案是 ulp 的倍数
    double approx(double val, const double ulp);
    // 以固定字符分割字符串
    void splitString(vector<string> &strList, const string &str, char ch);
}

#endif