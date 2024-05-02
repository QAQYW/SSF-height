#include <iostream>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>
#include <windows.h>
#include <unistd.h>

#include "resource.h"
#include "tools.h"
#include "dataGenerator.h"
#include "trajectory.h"
#include "energy.h"
#include "aco.h"
#include "acoOnline.h"
#include "naive.h"
#include "pso.h"
#include "ga.h"
#include "greedy.h"

/* ---------------------------- global variables ---------------------------- */

std::vector<std::string> filenames;

/* -------------------------------- functions ------------------------------- */

/// @brief 读存储文件名的文本文件
/// @param expNum 测试实例数量
/// @param dir 目录
/// @param onlineFileFormat 是否是online格式
void readFilename(int &expNum, std::string dir, bool onlineFileFormat) {
    std::ifstream fin;
    if (onlineFileFormat) {
        fin.open(dir + "\\online_filename_set.txt");
    } else {
        fin.open(dir + "filename_set.txt");
    }
    
    int num;
    fin >> num;
    // if (expNum <= 0 || expNum > num) {
    //     expNum = num;
    // }
    expNum = num;

    std::string filename = "";
    for (int i = 1; i <= num; i++) {
        fin >> filename;
        filenames.push_back(filename);
    }

    fin.close();
}

int main() {

    std::srand((unsigned int) std::time(NULL));


    return 0;
}