#include "tools.h"

double tools::randDouble(double minVal, double maxVal) {
    return (rand() / (double)RAND_MAX) * (maxVal - minVal) + minVal;
}

int tools::randInt(int minVal, int maxVal) {
    return (rand() % (maxVal - minVal + 1)) + minVal;
}

double tools::approx(double val, const double ulp) {
    // std::round()作用是四舍五入
    return std::round(val / ulp) * ulp;
}

void tools::splitString(vector<string> &strList, const string &str, char ch) {
    int len = str.size();
    if (len == 0) {
        return;
    }
    int pos = 0;
    for (int i = 0; i < len; i++) {
        if (str[i] == ch) {
            strList.push_back(str.substr(pos, i - pos));
            pos = i + 1;
        }
    }
}