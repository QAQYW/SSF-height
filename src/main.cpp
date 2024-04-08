#include <iostream>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>

#include "resource.h"
#include "tools.h"
#include "dataGenerator.h"
#include "aco.h"
#include "naive.h"
#include "acoOnline.h"
#include "trajectory.h"
#include "energy.h"

std::string direction = ".\\tiny_test";
int exampleNum = 1;
std::vector<unsigned int> seeds;
std::vector<std::string> filenames;

/// @brief 批量生成离线问题数据
/// @param expNum 数据样例数量
/// @param dir 样例存储目录
void generateData_Offline(int expNum, std::string dir) {
    if (expNum <= 0) {
        std::cout << "Input the number of test instances: ";
        std::cin >> expNum;
    }
    for (int i = 1; i <= expNum; i++) {
        unsigned int seed = rand();
        seeds.push_back(seed);
    }
    for (int i = 1; i <= expNum; i++) {
        DataGenerator dg(dir, 5);
        dg.generateAndSave(seeds[i - 1], i);
        std::cout << "instance " << std::to_string(i) << " generated.\n";
        std::string filename = dir + "\\" + dg.filenameBase + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }
    // 存下所有测试数据的文件名
    std::ofstream fout;
    fout.open(dir + "\\filename_set.txt");
    fout << std::to_string(expNum) << "\n";
    for (int i = 1; i <= expNum; i++) {
        fout << filenames[i - 1] << "\n";
    }
    fout.close();
    std::cout << "generateData_Offline\n";
}

/// @brief 批量生成在线问题数据
/// @param expNum 数据样例数量
/// @param dir 样例存储目录
void generateData_Online(int expNum, std::string dir) {
    if (expNum <= 0) {
        std::cout << "Input the number of test instances: ";
        std::cin >> expNum;
    }
    for (int i = 1; i <= expNum; i++) {
        unsigned int seed = rand();
        seeds.push_back(seed);
    }
    for (int i = 1; i <= expNum; i++) {
        DataGenerator dg(dir, 10);
        dg.generateAndSave_Online(seeds[i - 1], i);
        std::cout << "online instance " << std::to_string(i) << " has generated.\n";
        std::string filename = dir + "\\online_" + dg.filenameBase + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }
    // 存下所有测试数据的文件名
    std::ofstream fout;
    fout.open(dir + "\\online_filename_set.txt");
    fout << std::to_string(expNum) << "\n";
    for (int i = 1; i <= expNum; i++) {
        fout << filenames[i - 1] << "\n";
    }
    fout.close();
    std::cout << "generateData_Online\n";
}

/// @brief 蚁群算法求解在线问题
/// @param expNum 数据样例数量
/// @param dir 样例读取目录
void solve_Online_ACO(int expNum, std::string dir) {
    // Read filenames
    if (filenames.empty()) {
        std::ifstream fin;
        fin.open(dir + "\\online_filename_set.txt");
        int num;
        fin >> num;
        if (expNum <= 0 || expNum > num) expNum = num;
        for (int i = 1; i <= num; i++) {
            std::string filename;
            fin >> filename;
            filenames.push_back(filename);
        }
        fin.close();
    }
    // std::cout << "\ntest break point\n";

    for (int i = 1; i <= expNum; i++) {
        ProblemOnline2D prob2D;
        prob2D.initFromFile(filenames[i - 1]);
        // std::cout << "\nprob2D.initFromFile\n";

        ProblemOnlineDisc2D probDisc2D = ProblemOnlineDisc2D(prob2D);
        // std::cout << "\ncreate object of 'ProblemOnlineDisc2D'\n";

        online::ACOSolver_Online onlineSolver = online::ACOSolver_Online(&probDisc2D);
        // std::cout << "\ncreate online solver\n";

        std::vector<double> speedSche;
        onlineSolver.solve(speedSche);
        double cost = onlineSolver.getCost();
        double hcost = onlineSolver.getHcost();
        double vcost = onlineSolver.getVcost();
        Trajectory optTraj = onlineSolver.getTrajectory();

        // std::string filename = dir + "\\online_answer_aco_prop10_" + std::to_string(i) + ".txt";
        std::string filename = dir + "\\online_answer_aco_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
        std::ofstream fout;
        fout.open(filename);
        fout << "distance\tspeed\theight\n";
        int num = probDisc2D.getLengthDiscNum();
        for (int i = 0; i < num; i++) {
            double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
            double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
            fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
        }
        fout << " cost = " << std::to_string(cost) << "\n"; // << " or " << std::to_string(hcost + vcost) << "\n";
        fout << "hcost = " << std::to_string(hcost) << "\n";
        fout << "vcost = " << std::to_string(vcost) << "\n";
        fout.close();
        std::cout << "online aco: " << i << "/" << expNum << "\n";
    }
    std::cout << "solve_online_aco\n";
}

/// @brief 蚁群算法求解离线问题
/// @param expNum 数据样例数量
/// @param dir 样例读取目录
void solve_Offline_ACO(int expNum, std::string dir) {
    // Read filenames
    if (filenames.empty()) {
        std::ifstream fin;
        fin.open(dir + "\\filename_set.txt");
        int num;
        fin >> num;
        if (expNum <= 0 || expNum > num) {
            expNum = num;
        }
        for (int i = 1; i <= num; i++) {
            std::string filename;
            fin >> filename;
            filenames.push_back(filename);
        }
        fin.close();
    }

    for (int i = 1; i <= expNum; i++) {
        Problem2D prob2D;
        prob2D.initFromFile(filenames[i - 1]);
        ProblemDisc2D probDisc2D(prob2D);
        aco::ACOSolver acoSolver(&probDisc2D);
        acoSolver.solve();
        Trajectory optTraj = acoSolver.getTrajectory();

        std::vector<double> speedSche;
        double hcost = optTraj.calHeightCost();
        // double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
        double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
        // std::string filename = dir + "\\" + "offline_answer_aco_prop10_" + std::to_string(i) + ".txt";
        std::string filename = dir + "\\offline_answer_aco_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
        std::ofstream fout;
        fout.open(filename);
        fout << "distance\tspeed\theight\n";
        int num = probDisc2D.getLengthDiscNum();
        for (int i = 0; i < num; i++) {
            double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
            double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
            fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
        }
        fout << " cost = " << std::to_string(hcost + vcost) << "\n";
        fout << "hcost = " << std::to_string(hcost) << "\n";
        fout << "vcost = " << std::to_string(vcost) << "\n";
        fout.close();
        std::cout << "aco: " << i << "/" << expNum << "\n";
    }
    std::cout << "solve_Offline_ACO\n";
}

/// @brief dfs枚举所有路径+剪枝
/// @param expNum 数据样例数量
/// @param dir 样例读取目录
void solve_Offline_Naive(int expNum, std::string dir) {
    // Read filenames
    if (filenames.empty()) {
        std::ifstream fin;
        fin.open(dir + "\\filename_set.txt");
        int num;
        fin >> num;
        if (expNum <= 0 || expNum > num) {
            expNum = num;
        }
        for (int i = 1; i <= num; i++) {
            std::string filename;
            fin >> filename;
            filenames.push_back(filename);
        }
        fin.close();
    }

    for (int i = 1; i <= expNum; i++) {
        Problem2D prob2D;
        prob2D.initFromFile(filenames[i - 1]);
        ProblemDisc2D probDisc2D(prob2D);
        naive::NaiveSolver naiveSolver(&probDisc2D);
        naiveSolver.solve();
        Trajectory optTraj = naiveSolver.getTrajectory();

        std::vector<double> speedSche;
        double hcost = optTraj.calHeightCost();
        // double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
        double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj);
        // std::string filename = dir + "\\" + "offline_answer_naive_prop10_" + std::to_string(i) + ".txt";
        std::string filename = dir + "\\offline_answer_aco_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
        std::ofstream fout;
        fout.open(filename);
        fout << "distance\tspeed\theight\n";
        int num = probDisc2D.getLengthDiscNum();
        for (int i = 0; i < num; i++) {
            double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
            double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
            fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
        }
        fout << " cost = " << std::to_string(hcost + vcost) << "\n";
        fout << "hcost = " << std::to_string(hcost) << "\n";
        fout << "vcost = " << std::to_string(vcost) << "\n";
        fout.close();
        std::cout << "aco: " << i << "/" << expNum << "\n";
    }
    std::cout << "solve_Offline_Naive\n";
}

int main(int argc, char *argv[]) {

    std::srand((unsigned int) time(NULL));

    exampleNum = 2;
    direction = ".\\tiny_test";

    /* --------------------------------- Offline -------------------------------- */

    generateData_Offline(exampleNum, direction);
    solve_Offline_ACO(exampleNum, direction);
    solve_Offline_Naive(exampleNum, direction);

    /* --------------------------------- Online --------------------------------- */

    // generateData_Online(exampleNum, direction);
    // solve_Online_ACO(exampleNum, direction);
    
    
    // system("pause");
    return 0;
}