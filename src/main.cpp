// #include <iostream>
// #include <ctime>
// #include <cstdlib>
// #include <string>
// #include <vector>
// // #include <filesystem> // need c++ 17
// #include <windows.h>
// #include <unistd.h>

// #include "resource.h"
// #include "tools.h"
// #include "dataGenerator.h"
// #include "aco.h"
// #include "naive.h"
// #include "acoOnline.h"
// #include "trajectory.h"
// #include "energy.h"
// #include "pso.h"
// #include "ga.h"
// #include "greedy.h"

// std::string direction = ".\\tiny_test";
// int exampleNum = 1;
// std::vector<unsigned int> seeds;
// std::vector<std::string> filenames;

// void readFilename(int &expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     std::ifstream fin;
//     if (onlineFile) {
//         fin.open(dir + "\\online_filename_set.txt");
//     } else {
//         fin.open(dir + "\\filename_set.txt");
//     }
//     int num;
//     fin >> num;
//     if (expNum <= 0 || expNum > num) {
//         expNum = num;
//         exampleNum = num;
//     }
//     std::string filename = "";
//     for (int i = 1; i <= num; i++) {
//         fin >> filename;
//         filenames.push_back(filename);
//     }
//     fin.close();
// }

// /// @brief 批量生成离线问题数据
// /// @param expNum 数据样例数量
// /// @param dir 样例存储目录
// /// @param senNum 传感器数量（若输入0，则视为默认值5）
// void generateData_Offline(int expNum, std::string dir, int senNum) {
//     if (expNum <= 0) {
//         std::cout << "Input the number of test instances: ";
//         std::cin >> expNum;
//     }
//     for (int i = 1; i <= expNum; i++) {
//         unsigned int seed = rand();
//         seeds.push_back(seed);
//     }
//     if (senNum <= 0) senNum = 5;
//     for (int i = 1; i <= expNum; i++) {
//         DataGenerator dg(dir, senNum);
//         dg.generateAndSave(seeds[i - 1], i);
//         std::cout << "instance " << std::to_string(i) << " generated.\n";
//         std::string filename = dir + "\\" + dg.filenameBase + std::to_string(i) + ".txt";
//         filenames.push_back(filename);
//     }
//     // 存下所有测试数据的文件名
//     std::ofstream fout;
//     fout.open(dir + "\\filename_set.txt");
//     fout << std::to_string(expNum) << "\n";
//     for (int i = 1; i <= expNum; i++) {
//         fout << filenames[i - 1] << "\n";
//     }
//     fout.close();
//     std::cout << "generateData_Offline\n";
// }

// /// @brief 批量生成在线问题数据
// /// @param expNum 数据样例数量
// /// @param dir 样例存储目录
// /// @param senNum 传感器数量（若输入0，则视为默认值5）
// void generateData_Online(int expNum, std::string dir, int senNum) {
//     if (expNum <= 0) {
//         std::cout << "Input the number of test instances: ";
//         std::cin >> expNum;
//     }
//     for (int i = 1; i <= expNum; i++) {
//         unsigned int seed = rand();
//         seeds.push_back(seed);
//     }
//     if (senNum <= 0) senNum = 5;
//     for (int i = 1; i <= expNum; i++) {
//         DataGenerator dg(dir, senNum);
//         dg.generateAndSave_Online(seeds[i - 1], i);
//         std::cout << "online instance " << std::to_string(i) << " has generated.\n";
//         std::string filename = dir + "\\online_" + dg.filenameBase + std::to_string(i) + ".txt";
//         filenames.push_back(filename);
//     }
//     // 存下所有测试数据的文件名
//     std::ofstream fout;
//     fout.open(dir + "\\online_filename_set.txt");
//     fout << std::to_string(expNum) << "\n";
//     for (int i = 1; i <= expNum; i++) {
//         fout << filenames[i - 1] << "\n";
//     }
//     fout.close();
//     std::cout << "generateData_Online\n";
// }

// /// @brief 蚁群算法求解在线问题
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// void solve_Online_ACO(int expNum, std::string dir) {
//     // Read filenames
//     if (filenames.empty()) {
//         std::ifstream fin;
//         fin.open(dir + "\\online_filename_set.txt");
//         int num;
//         fin >> num;
//         if (expNum <= 0 || expNum > num) {
//             expNum = num;
//             exampleNum = num;
//         }
//         for (int i = 1; i <= num; i++) {
//             std::string filename;
//             fin >> filename;
//             filenames.push_back(filename);
//         }
//         fin.close();
//     }
//     // std::cout << "\ntest break point\n";

//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         ProblemOnline2D prob2D;
//         prob2D.initFromOnlineFile(filenames[i - 1]);
//         // std::cout << "\nprob2D.initFromOnlineFile\n";

//         ProblemOnlineDisc2D probDisc2D = ProblemOnlineDisc2D(prob2D);
//         // std::cout << "\ncreate object of 'ProblemOnlineDisc2D'\n";

//         online::ACOSolver_Online onlineSolver = online::ACOSolver_Online(&probDisc2D);
//         // std::cout << "\ncreate online solver\n";

//         std::vector<double> speedSche;
//         onlineSolver.solve(speedSche);
//         double cost = onlineSolver.getCost();
//         double hcost = onlineSolver.getHcost();
//         double vcost = onlineSolver.getVcost();
//         Trajectory optTraj = onlineSolver.getTrajectory();

//         // std::string filename = dir + "\\online_answer_aco_prop10_" + std::to_string(i) + ".txt";
//         std::string filename = dir + "\\online_answer_aco_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(cost) << "\n"; // << " or " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "online aco: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_online_aco\n";
// }

// /// @brief 蚁群算法求解离线问题
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// /// @param onlineFile true则读在线样例，false则读离线样例
// void solve_Offline_ACO(int expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     if (filenames.empty()) {
//         std::ifstream fin;
//         if (onlineFile) {
//             fin.open(dir + "\\online_filename_set.txt");
//         } else {
//             fin.open(dir + "\\filename_set.txt");
//         }
//         int num;
//         fin >> num;
//         if (expNum <= 0 || expNum > num) {
//             expNum = num;
//             exampleNum = num;
//         }
//         for (int i = 1; i <= num; i++) {
//             std::string filename;
//             fin >> filename;
//             filenames.push_back(filename);
//         }
//         fin.close();
//     }

//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         Problem2D prob2D;
//         if (onlineFile) {
//             prob2D.initFromOnlineFile(filenames[i - 1]);
//         } else {
//             prob2D.initFromFile(filenames[i - 1]);
//         }
//         ProblemDisc2D probDisc2D(prob2D);
//         aco::ACOSolver acoSolver(&probDisc2D);
//         acoSolver.solve();
//         Trajectory optTraj = acoSolver.getTrajectory();

//         std::vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         // double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
//         double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
//         // std::string filename = dir + "\\" + "offline_answer_aco_prop10_" + std::to_string(i) + ".txt";
//         std::string filename = dir + "\\offline_answer_aco_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "aco: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_Offline_ACO\n";
// }

// /// @brief dfs枚举所有路径+剪枝
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// /// @param onlineFile true则读在线样例，false则读离线样例
// void solve_Offline_Naive(int expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     if (filenames.empty()) {
//         std::ifstream fin;
//         if (onlineFile) {
//             fin.open(dir + "\\online_filename_set.txt");
//         } else {
//             fin.open(dir + "\\filename_set.txt");
//         }
//         int num;
//         fin >> num;
//         if (expNum <= 0 || expNum > num) {
//             expNum = num;
//             exampleNum = num;
//         }
//         for (int i = 1; i <= num; i++) {
//             std::string filename;
//             fin >> filename;
//             filenames.push_back(filename);
//         }
//         fin.close();
//     }

//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         Problem2D prob2D;
//         if (onlineFile) {
//             prob2D.initFromOnlineFile(filenames[i - 1]);
//         } else {
//             prob2D.initFromFile(filenames[i - 1]);
//         }
//         ProblemDisc2D probDisc2D(prob2D);
//         naive::NaiveSolver naiveSolver(&probDisc2D);
//         naiveSolver.solve();
//         Trajectory optTraj = naiveSolver.getTrajectory();

//         std::vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         // double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
//         double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
//         // std::string filename = dir + "\\" + "offline_answer_naive_prop10_" + std::to_string(i) + ".txt";
//         std::string filename = dir + "\\offline_answer_naive_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "naive: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_Offline_Naive\n";
// }

// /// @brief 离散粒子群DPSO算法求解离线问题
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// /// @param onlineFile true则读在线样例，false则读离线样例
// void solve_Offline_PSO(int expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     if (filenames.empty()) readFilename(expNum, dir, onlineFile);
    
//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         Problem2D prob2D;
//         if (onlineFile) {
//             prob2D.initFromOnlineFile(filenames[i - 1]);
//         } else {
//             prob2D.initFromFile(filenames[i - 1]);
//         }
//         ProblemDisc2D probDisc2D(prob2D);
//         pso::PSOSolver psoSolver(&probDisc2D);
//         psoSolver.solve();
//         Trajectory optTraj = psoSolver.getTrajectory();

//         std::vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
//         std::string filename = dir + "\\offline_answer_pso_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "pso: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_Offline_PSO\n";
// }

// /// @brief 遗传算法求解离线问题
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// /// @param onlineFile true则读在线样例，false则读离线样例
// void solve_Offline_GA(int expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     if (filenames.empty()) readFilename(expNum, dir, onlineFile);

//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         Problem2D prob2D;
//         if (onlineFile) {
//             prob2D.initFromOnlineFile(filenames[i - 1]);
//         } else {
//             prob2D.initFromFile(filenames[i - 1]);
//         }
//         ProblemDisc2D probDisc2D(prob2D);
//         ga::GASolver gaSolver(&probDisc2D);
//         gaSolver.solve();
//         Trajectory optTraj = gaSolver.getTrajectory();

//         std::vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
//         std::string filename = dir + "\\offline_answer_ga_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "ga: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_Offline_GA\n";
// }

// /// @brief 贪心方法求解离线问题
// /// @param expNum 数据样例数量
// /// @param dir 样例读取目录
// /// @param onlineFile true则读在线样例，false则读离线样例
// void solve_Offline_Greedy(int expNum, std::string dir, bool onlineFile) {
//     // Read filenames
//     if (filenames.empty()) readFilename(expNum, dir, onlineFile);

//     for (int i = 1; i <= expNum; i++) {
//         std::clock_t sc = std::clock();
//         Problem2D prob2D;
//         if (onlineFile) {
//             prob2D.initFromOnlineFile(filenames[i - 1]);
//         } else {
//             prob2D.initFromFile(filenames[i - 1]);
//         }
//         ProblemDisc2D probDisc2D(prob2D);
//         greedy::GreedySolver greedySolver(&probDisc2D);
//         greedySolver.solve();
//         Trajectory optTraj = greedySolver.getTrajectory();

//         std::vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         double vcost = energy_calculator::calSpeedCost(probDisc2D, optTraj, speedSche);
//         std::string filename = dir + "\\offline_answer_greedy_prop" + std::to_string((int) resource::HEIGHT_COST_PROPOR) + "_" + std::to_string(i) + ".txt";
//         std::ofstream fout;
//         fout.open(filename);
//         fout << "distance\tspeed\theight\n";
//         int num = probDisc2D.getLengthDiscNum();
//         for (int i = 0; i < num; i++) {
//             double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
//             double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
//             fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
//         }
//         fout << " cost = " << std::to_string(hcost + vcost) << "\n";
//         fout << "hcost = " << std::to_string(hcost) << "\n";
//         fout << "vcost = " << std::to_string(vcost) << "\n";
//         fout.close();
//         std::clock_t ec = std::clock();
//         double runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
//         std::cout << "runtime: " << std::to_string(runtime) << "\n";
//         std::cout << "greedy: " << i << "/" << expNum << "\n";
//     }
//     std::cout << "solve_Offline_Greedy\n";
// }

// /// @brief 将当前时间（精确到秒）转换为字符串，用下划线 "_" 连接
// /// @return 表示时间的字符串
// std::string getTimeString() {
//     time_t now;
//     // 获取1900年1月1日0点0分0秒到现在经过的秒数
//     time(&now);
//     // 将秒数转换为本地时间
//     // 年份从1900年算起，所以要+1900
//     // 月份为0~11，所以要+1
//     tm *local = localtime(&now);
//     std::string str = std::to_string(local->tm_year + 1900) + "_"
//         + std::to_string(local->tm_mon + 1) + "_"
//         + std::to_string(local->tm_mday) + "_"
//         + std::to_string(local->tm_hour) + "_"
//         + std::to_string(local->tm_min) + "_"
//         + std::to_string(local->tm_sec);
    
//     std::cout << "\n\ttimestr = " << str << "\n\n";
//     return str;
// }

// void mk_dir(std::string dir, std::string timestr) {
//     std::wstring wstr(dir.begin(), dir.end());
//     LPCWSTR p = wstr.c_str();
//     CreateDirectoryW(p, NULL);

//     std::ofstream fout;
//     fout.open(".\\tiny_test\\timestr.txt");
//     fout << timestr << "\n";
//     fout.close();
// }



// int main(int argc, char *argv[]) {

//     std::srand((unsigned int) time(NULL));

//     // direction = ".\\tiny_test";

//     const std::string special_data_timestr = "1900_1_1_0_0_0";
//     std::string timestr = "";

//     int opt1, opt2;
//     std::cout << "\nproblem types:\n";
//     std::cout << "    1: offline\n";
//     std::cout << "    2: online\n";
//     std::cout << "\nchoose option: ";
//     // std::cin >> opt1;
//     opt1 = 2;
//     std::cout << "\noptions:\n";
//     std::cout << "    1: use newly generated data\n";
//     std::cout << "    2: use existed data\n";
//     std::cout << "    3: use special data (1900_1_1_0_0_0)\n";
//     std::cout << "\nchoose option: ";
//     // std::cin >> opt2;
//     opt2 = 1;

//     filenames.clear();

//     int sensorNum = 5; //5;

//     if (opt1 == 1) {
//         // Offline
//         switch (opt2) {
//             case 1:
//                 timestr = getTimeString();
//                 direction = ".\\tiny_test\\" + timestr;
//                 mk_dir(direction, timestr);
//                 exampleNum = 5;
//                 // generateData_Offline(exampleNum, direction, 5);
//                 generateData_Offline(exampleNum, direction, sensorNum);
//                 solve_Offline_ACO(exampleNum, direction, false);
//                 solve_Offline_PSO(exampleNum, direction, false);
//                 solve_Offline_GA(exampleNum, direction, false);
//                 solve_Offline_Greedy(exampleNum, direction, false);
//                 // if (sensorNum > 5) break;
//                 // solve_Offline_Naive(exampleNum, direction, false);
//                 break;
//             case 2:
//                 std::cout << "\ninput folder name (time string): ";
//                 std::cin >> timestr;
//                 exampleNum = 0;
//                 direction = ".\\tiny_test\\" + timestr;
//                 solve_Offline_ACO(exampleNum, direction, false);
//                 solve_Offline_PSO(exampleNum, direction, false);
//                 solve_Offline_GA(exampleNum, direction, false);
//                 solve_Offline_Greedy(exampleNum, direction, false);
//                 solve_Offline_Naive(exampleNum, direction, false);
//                 break;
//             case 3:
//                 timestr = special_data_timestr;
//                 direction = ".\\tiny_test\\" + timestr;
//                 exampleNum = 0;
//                 solve_Offline_ACO(exampleNum, direction, false);
//                 solve_Offline_PSO(exampleNum, direction, false);
//                 solve_Offline_GA(exampleNum, direction, false);
//                 solve_Offline_Greedy(exampleNum, direction, false);
//                 solve_Offline_Naive(exampleNum, direction, false);
//                 break;
//         }
//     } else if (opt1 == 2) {
//         // Online
//         switch (opt2) {
//             case 1:
//                 timestr = getTimeString();
//                 direction = ".\\tiny_test\\" + timestr;
//                 mk_dir(direction, timestr);
//                 exampleNum = 5;
//                 generateData_Online(exampleNum, direction, 5);
//                 solve_Online_ACO(exampleNum, direction);
//                 solve_Offline_ACO(exampleNum, direction, true);
//                 solve_Offline_Naive(exampleNum, direction, true);
//                 break;
//             case 2:
//                 std::cout << "\ninput folder name (time string): ";
//                 std::cin >> timestr;
//                 exampleNum = 0;
//                 direction = ".\\tiny_test\\" + timestr;
//                 solve_Online_ACO(exampleNum, direction);
//                 solve_Offline_ACO(exampleNum, direction, true);
//                 solve_Offline_Naive(exampleNum, direction, true);
//                 break;
//             case 3:
//                 timestr = special_data_timestr;
//                 direction = ".\\tiny_test\\" + timestr;
//                 exampleNum = 0;
//                 solve_Online_ACO(exampleNum, direction);
//                 solve_Offline_ACO(exampleNum, direction, true);
//                 solve_Offline_Naive(exampleNum, direction, true);
//                 break;
//         }
//     }
    
//     // system("pause");
//     return 0;
// }