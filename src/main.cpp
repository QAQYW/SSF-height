#include <iostream>
#include <ctime>
#include <cstdlib>

#include "resource.h"
#include "problemSet.h"
#include "tools.h"
#include "dataGenerator.h"
#include "aco.h"
#include "naive.h"

string direction = ".\\tiny_test";
int exampleNum = 1;
vector<unsigned int> seeds;
vector<string> filenames;

/**
 * 批量生成数据
 * exampleNum: 数据个数
 * dir: 数据存储目录
*/
void generateData(int exampleNum, string dir) {
    if (exampleNum <= 0) {
        std::cout << "Input the number of test instances: ";
        std::cin >> exampleNum;
    }
    for (int i = 1; i <= exampleNum; i++) {
        unsigned int seed = rand();
        seeds.push_back(seed);
    }
    for (int i = 1; i <= exampleNum; i++) {
        DataGenerator dg(dir, 5);
        dg.generateAndSave(seeds[i - 1], i);
        std::cout << "instance " << std::to_string(i) << " generated.\n";
        string filename = dir + "\\" + dg.filenameBase + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }
    // 存下所有测试数据的文件名
    ofstream fout;
    fout.open(dir + "\\filename_set.txt");
    fout << std::to_string(exampleNum) << "\n";
    for (int i = 1; i <= exampleNum; i++) {
        fout << filenames[i - 1] << "\n";
    }
    fout.close();
    cout << "generateData\n";
}

void solve_ACO(int exampleNum, string dir) {
    // Read filenames
    if (filenames.empty()) {
        ifstream fin;
        fin.open(dir + "\\filename_set.txt");
        int num;
        fin >> num;
        if (exampleNum <= 0 || exampleNum > num) {
            exampleNum = num;
        }
        for (int i = 1; i <= num; i++) {
            string filename;
            fin >> filename;
            filenames.push_back(filename);
        }
        fin.close();
    }

    for (int i = 1; i <= exampleNum; i++) {
        Problem2D prob2D;
        prob2D.initFromFile(filenames[i - 1]);
        ProblemDisc2D probDisc2D(prob2D);
        aco::ACOSolver acoSolver(&probDisc2D);
        acoSolver.solve();
        aco::Trajectory optTraj = acoSolver.getTrajectory();

        vector<double> speedSche;
        double hcost = optTraj.calHeightCost();
        double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
        string filename = dir + "\\" + "answer_aco_prop10_" + std::to_string(i) + ".txt";
        ofstream fout;
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
        cout << "aco: " << i << "/" << exampleNum << "\n";
    }
    cout << "solve_ACO\n";
}

void solve_Naive(int exampleNum, string dir) {
    // Read filenames
    if (filenames.empty()) {
        ifstream fin;
        fin.open(dir + "\\filename_set.txt");
        int num;
        fin >> num;
        if (exampleNum <= 0 || exampleNum > num) {
            exampleNum = num;
        }
        for (int i = 1; i <= num; i++) {
            string filename;
            fin >> filename;
            filenames.push_back(filename);
        }
        fin.close();
    }

    for (int i = 1; i <= exampleNum; i++) {
        Problem2D prob2D;
        prob2D.initFromFile(filenames[i - 1]);
        ProblemDisc2D probDisc2D(prob2D);
        naive::NaiveSolver naiveSolver(&probDisc2D);
        naiveSolver.solve();
        aco::Trajectory optTraj = naiveSolver.getTrajectory();

        vector<double> speedSche;
        double hcost = optTraj.calHeightCost();
        double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
        
        string filename = dir + "\\" + "answer_naive_prop10_" + std::to_string(i) + ".txt";
        ofstream fout;
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
        cout << "naive: " << i << "/" << exampleNum << "\n";
    }
    cout << "solve_Naive\n";
}

// void generateData_Solve(int exampleNum, bool generateFlag, bool solveFlag) {
//     /* --------------------------------- 批量生成数据 --------------------------------- */
//     if (exampleNum <= 0) {
//         std::cout << "Input the number of test instances: ";
//         std::cin >> exampleNum;
//     }
//     for (int i = 1; i <= exampleNum; i++) {
//         unsigned int seed = rand();
//         seeds.push_back(seed);
//     }
//     for (int i = 1; i <= exampleNum; i++) {
//         DataGenerator dg(dir, 30);
//         if (generateFlag) {
//             dg.generateAndSave(seeds[i - 1], i);
//             std::cout << "instance " << std::to_string(i) << " generated.\n";
//         }
//         string filename = dir + "\\" + dg.filenameBase + std::to_string(i) + ".txt";
//         filenames.push_back(filename);
//     }
//     /* ----------------------------------- 求解 ----------------------------------- */
//     if (!solveFlag) return;
//     for (int exampleIndex = 1; exampleIndex <= exampleNum; exampleIndex++) {
//         Problem2D prob2D;
//         prob2D.initFromFile(filenames[exampleIndex - 1]);
//         ProblemDisc2D probDisc2D(prob2D);
//         aco::ACOSolver acoSolver(&probDisc2D);
//         acoSolver.solve();
//         aco::Trajectory optTraj = acoSolver.getTrajectory();
//         vector<double> speedSche;
//         double hcost = optTraj.calHeightCost();
//         double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
//         string filename = dir + "\\" + "answer_aco_prop10_" + std::to_string(exampleIndex) + ".txt";
//         ofstream fout;
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
//     }
// }

int main(int argc, char *argv[]) { // int main() {

    srand((unsigned int) time(NULL));

    exampleNum = 10;
    direction = ".\\tiny_test";

    generateData(exampleNum, direction);
    solve_ACO(exampleNum, direction);
    solve_Naive(exampleNum, direction);

    // generateData(10);
    // generateData_Solve(10, false, true);
    
    // system("pause");
    return 0;
}