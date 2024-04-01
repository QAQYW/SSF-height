#include "tools.h"

double tools::randDouble(double minVal, double maxVal) {
    return (std::rand() / (double) RAND_MAX) * (maxVal - minVal) + minVal;
}

int tools::randInt(int minVal, int maxVal) {
    return (std::rand() % (maxVal - minVal + 1)) + minVal;
}

double tools::approx(double val, const double ulp) {
    return std::round(val / ulp) * ulp;
}

void tools::splitString(std::vector<std::string> &strList, const std::string &str, char ch) {
    int len = str.size();
    if (len == 0) return;
    int pos = 0;
    for (int i = 0; i < len; i++) {
        if (str[i] == ch) {
            strList.push_back(str.substr(pos, i - pos));
            pos = i + 1;
        }
    }
}