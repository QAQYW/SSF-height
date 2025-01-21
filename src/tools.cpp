#include "tools.h"
#include <iostream>
#include <fstream>

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

void tools::printVector(const std::string &path, const std::vector<double> &vec) {
    std::ofstream fout;
    fout.open(path, std::ios::out | std::ios::app);
    for (double val : vec) {
        fout << val << "\t";
    }
    fout << "\n";
    fout.close();
}

void tools::printVector(const std::string &msg, const std::string &path, const std::vector<double> &vec) {
    std::ofstream fout;
    fout.open(path, std::ios::out | std::ios::app);
    fout << msg << "\n";
    fout.close();
    printVector(path, vec);
}
