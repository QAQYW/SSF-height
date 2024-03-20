#include <iostream>
#include <ctime>
#include <cstdlib>

#include "resource.h"
#include "problemSet.h"
#include "tools.h"
#include "dataGenerator.h"

const string dir = ".\\test";
int exampleNum = 1;
vector<unsigned int> seeds;
vector<string> filenames;

int main(int argc, char *argv[]) {
    /* --------------------------------- 批量生成数据 --------------------------------- */
    srand(time(0));

    // std::cout << "Input the number of test instances: ";
    // std::cin >> exampleNum;
    for (int i = 1; i <= exampleNum; i++) {
        unsigned int seed = rand();
        seeds.push_back(seed);
    }
    for (int i = 1; i <= exampleNum; i++) {
        DataGenerator dg(dir, 30);
        dg.generateAndSave(seeds[i - 1], i);
        std::cout << "instance " << std::to_string(i) << " generated.\n";
        string filename = dir + "\\" + dg.filenameBase + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }

    /* ----------------------------------- 求解 ----------------------------------- */
    for (int exampleIndex = 1; exampleIndex <= exampleNum; exampleIndex++) {
        Problem2D prob2D;
        prob2D.initFromFile(filenames[exampleIndex - 1]);
        // cout << "successful here\n";
        // prob2D.initFromFile(".\\test\\test_data_1.txt");
        ProblemDisc2D probDisc2D(prob2D);
        aco::ACOSolver acoSolver(&probDisc2D);
        acoSolver.solve();
        aco::Trajectory* optTraj = acoSolver.getTrajectory();
        vector<double> speedSche;
        double hcost = optTraj->calHeightCost();
        double vcost = optTraj->calSpeedCost(probDisc2D, speedSche);
        string filename = dir + "\\" + "answer_" + std::to_string(exampleIndex) + ".txt";
        ofstream fout;
        fout.open(filename);
        int num = probDisc2D.getLengthDiscNum();
        fout << "distance\tspeed\theight\n";
        for (int i = 0; i < num; i++) {
            double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
            double hei = resource::indexToHeight(i, prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
            fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
        }
        fout.close();
    }
    return 0;
}