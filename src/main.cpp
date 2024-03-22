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

// int main(int argc, char *argv[]) {
int main() {
    /* --------------------------------- 批量生成数据 --------------------------------- */
    srand((unsigned int) time(NULL));

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
        // cout << "successful initFromFile()\n";
        // prob2D.initFromFile(".\\test\\test_data_1.txt");
        ProblemDisc2D probDisc2D(prob2D);
        // cout << "successful transforming to ProblemDisc2D\n";
        aco::ACOSolver acoSolver(&probDisc2D);
        // cout << "successful init ACOSolver\n";
        acoSolver.solve();
        // cout << "successful solve()\n";
        // if (acoSolver.getTrajectory() == nullptr) {
        //     cout << "empty pointer\n";
        // }
        aco::Trajectory optTraj = acoSolver.getTrajectory();

        // cout << "get optimal trajectory\n";

        vector<double> speedSche;
        double hcost = optTraj.calHeightCost();
        // cout << "1\n";
        double vcost = optTraj.calSpeedCost(probDisc2D, speedSche);
        // cout << "2\n";
        string filename = dir + "\\" + "answer_" + std::to_string(exampleIndex) + ".txt";
        ofstream fout;
        fout.open(filename);

        // cout << "opening file successfully\n";

        int num = probDisc2D.getLengthDiscNum();
        fout << "distance\tspeed\theight\n";
        // cout << "size of height list: " << std::to_string(optTraj.getHeightSche().size()) << "\n";
        // cout << "size of speed schedule: " << std::to_string(speedSche.size()) << "\n";
        // cout << "num = " << std::to_string(num) << "\n";

        // cout << "saving to file\n";

        for (int i = 0; i < num; i++) {
            double dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
            double hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob2D.getMinHeight(), resource::REF_UNIT_HEIGHT);
            fout << std::to_string(dis) << "\t" << std::to_string(speedSche[i]) << "\t" << std::to_string(hei) << "\n";
        }
        fout.close();
        // cout << "file closed\n";
    }
    system("pause");
    return 0;
}