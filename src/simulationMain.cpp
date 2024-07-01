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
#include "problem2D.h"
#include "problemDisc2D.h"
#include "problemOnline2D.h"
#include "problemOnlineDisc2D.h"

/* ---------------------------- global variables ---------------------------- */

struct Result {
    double cost;
    double hcost;
    double vcost;
    double runtime;
    std::string str;
};

std::vector<std::string> features; // 每个测试数据的特征值
std::vector<std::string> filenames;
std::vector<Result> results;

/// @brief 参数集合
namespace para {
    /* -------------------------- algorithm comparison -------------------------- */

    // 传感器数量，参考值 {5, 10, 20, 30, 40, 50} {5, 10, 15, 20, 25, 30}
    const int sensor_nums[] = {5};

    // 水滴曲线最大高度（米），参考值 {70, 100, 130, 160}
    const double max_y_mults[] = {70, 100, 130, 160};

    // 水滴曲线最大宽度（米），参考值 {30, 40, 50, 60}
    const double max_x_milts[] = {30, 40, 50, 60};

    // // 水滴最宽处宽度与路径总长度线性相关的系数，参考值 {0.2, 0.3, 0.4, 0.5, 0.6}
    // const double max_x_mult_coefs[] = {0.2, 0.3, 0.4, 0.5, 0.6};

    // // 传输时间与传输范围大小的平方相关的系数，参考值 {0.05, 0.1, 0.2, 0.3}
    // const double max_time_range_props[] = {0.05, 0.1, 0.2, 0.3};

    // 传输时间与传输范围大小线性相关的系数，参考值 {0.5, 1, 1.5, 2}
    const double time_props[] = {0.5, 1, 1.5, 2};

    // 最大膨胀系数，参考值 {1, 1.5, 2, 2.5, 3}
    const double max_swells[] = {1, 1.5, 2, 2.5, 3};

    /* ------------------------------- calibration ------------------------------ */

    // 蚁群蒸发系数
    const double evaporate_coefs[] = {0.1};//, 0.2, 0.3, 0.4, 0.5};

    // alpha
    const double alphas[] = {1, 2, 3, 4};
    // beta
    const double betas[] = {0, 1, 2, 3, 4, 5};
    // best performance: alpha=1, beta=2

    /* ---------------------------------- other --------------------------------- */

    // 算法名字
    const std::string algorithm_names[9] = {
        "DFS",
        "ACO",
        "PSO",
        "GA",
        "SA",
        "Greedy",
        "ACO-Online",
        "ACO-dominated",  // for calibration
        "ACO-normal"      // for calibration
    };
    // 算法枚举类型
    enum Algorithm {
        DFS = 0,
        ACO = 1,
        PSO = 2,
        GA = 3,
        SA = 4,
        Greedy = 6,
        ACO_Online = 6,
        ACO_dominated = 7,  // for calibration
        ACO_normal = 8      // for calibration
    };
    
} // namespace para

/* -------------------------------- functions ------------------------------- */

/// @brief 批量生成online格式的测试数据
/// @param dir 目录
/// @param online_file_format 是否online格式
/// @param num 每个参数组和生成num个数据
void generate_online_data(std::string dir, bool online_file_format, int num) {
    filenames.clear();
    features.clear();
    // 测试数据数量
    int count_data = 0;
    std::vector<unsigned int> seeds;

    for (int sensor_num : para::sensor_nums) {
        for (double max_y_mult : para::max_y_mults) {
            // for (double max_x_mult_coef : para::max_x_mult_coefs) {
            for (double max_x_mult : para::max_x_milts) {
                // for (double max_time_range_prop : para::max_time_range_props) {
                for (double time_prop : para::time_props) {
                    for (double max_swell : para::max_swells) {
                        for (int i = 1; i <= num; i++) {
                            ++count_data;
                            // DataGenerator dg = DataGenerator(dir, sensor_num, max_y_mult, max_x_mult_coef, max_time_range_prop, max_swell);
                            // DataGenerator dg = DataGenerator(dir, sensor_num, max_y_mult, max_x_mult_coef, time_prop, max_swell);
                            // DataGenerator dg = DataGenerator(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell);
                            DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell);
                            // 随机种子
                            unsigned int seed = std::rand();
                            seeds.push_back(seed);
                            // dg.generateAndSave_Online(seed, count_data);
                            dg2.generate_save_online(seed, count_data);
                            // 测试数据特征值
                            std::string feature = std::to_string(count_data) + "\t"
                                + std::to_string(sensor_num) + "\t"
                                + std::to_string(max_y_mult) + "\t"
                                + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
                                + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
                                + std::to_string(max_swell);
                            features.push_back(feature);
                            // std::cout << feature << "\n";
                            // 记录文件名
                            // std::string filename = dir + "\\online_" + dg.filenameBase + std::to_string(count_data) + ".txt";
                            std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
                            filenames.push_back(filename);
                        }
                    }
                }
            }
        }
    }
    
    std::ofstream fout;

    // 保存测试数据特征值
    fout.open(dir + "\\features.txt");
    for (int i = 0; i < count_data; i++) {
        fout << features[i] << "\n";
        // std::cout << features[i] << "\n";
    }
    fout.close();

    // 保存所有文件名
    fout.open(dir + "\\online_filename_set.txt");
    fout << std::to_string(count_data) << "\n";
    for (int i = 0; i < count_data; i++) {
        fout << filenames[i] << "\n";
    }
    fout.close();

    std::cout << "Generate " << count_data << " instances\n\n";
}

// void test20240622(std::string dir, pso::PSOSolver solver) {
//     int lengthDiscNum = 169;
//     Trajectory traj = Trajectory(lengthDiscNum);
//     std::ifstream fin;
//     fin.open(dir + "\\answer_PSO_3.txt");
//     double dis, spd, hei;
//     std::string tempstr = "";
//     std::getline(fin, tempstr);
//     for (int i = 0; i < lengthDiscNum; i++) {
//         fin >> dis >> spd >> hei;
//         traj.setHeightIndex(i, (int) ((hei - 80) / 10));
//     }
//     fin.close();
//     if (solver.isFeasible(traj)) {
//         std::cout << "Feasible Trajectory\n";
//     } else {
//         std::cout << "Infeasible Trajectory\n";
//     }
// }

/// @brief 读入filenames.txt和features.txt
/// @param instance_num 
/// @param dir 
/// @param online_file_format 是否online格式
void readInit(int &instance_num, std::string dir, bool online_file_format) {
    std::ifstream fin;
    std::string buff = "";

    // 读filenames.txt
    filenames.clear();
    fin.open(dir + "\\online_filename_set.txt");
    fin >> instance_num;
    for (int i = 0; i < instance_num; i++) {
        fin >> buff;
        filenames.push_back(buff);
    }
    fin.close();

    // 读features.txt
    fin.open(dir + "\\features.txt");
    for (int i = 0; i < instance_num; i++) {
        // fin >> buff;
        std::getline(fin, buff);
        features.push_back(buff);
    }
    fin.close();
}

/// @brief 用指定算法求解，然后保存结果
/// @param prob2d 
/// @param algorithm 
void solve(ProblemDisc2D &prob, para::Algorithm alg, std::string dir, int data_index) {
    // std::cout << "enter func solve()\n";
    std::clock_t sc = std::clock();
    Trajectory optTraj;
    if (alg == para::Algorithm::DFS) {
        // 0
        naive::NaiveSolver naiveSolver = naive::NaiveSolver(&prob);
        naiveSolver.solve();
        optTraj = naiveSolver.getTrajectory();
    } else if (alg == para::Algorithm::ACO || alg == para::Algorithm::ACO_dominated || alg == para::Algorithm::ACO_normal) {
        // 1
        aco::ACOSolver acoSolver = aco::ACOSolver(&prob);
        acoSolver.solve();
        optTraj = acoSolver.getTrajectory();
    } else if (alg == para::Algorithm::PSO) {
        // 2
        pso::PSOSolver psoSolver = pso::PSOSolver(&prob);
        psoSolver.solve();
        optTraj = psoSolver.getTrajectory();
        // if (data_index == 2) {
        //     test20240622(dir, psoSolver);
        // }
    } else if (alg == para::Algorithm::GA) {
        // 3
        ga::GASolver gaSolver = ga::GASolver(&prob);
        gaSolver.solve();
        optTraj = gaSolver.getTrajectory();
    } else if (alg == para::Algorithm::Greedy) {
        // 4
        greedy::GreedySolver greedySolver = greedy::GreedySolver(&prob);
        greedySolver.solve();
        optTraj = greedySolver.getTrajectory();
    }
    std::clock_t ec = std::clock();
    // 记录结果
    Result result;
    std::vector<double> speedSche;
    result.hcost = optTraj.calHeightCost(resource::HEIGHT_COST_PROPOR);
    result.vcost = energy_calculator::calSpeedCost(prob, optTraj, speedSche);
    result.cost = result.hcost + result.vcost;
    result.runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
    result.str = std::to_string(result.cost) + "\t" + std::to_string(result.hcost) + "\t" + std::to_string(result.vcost) + "\t" + std::to_string(result.runtime);
    results.push_back(result);
    // 保存结果（追加写入）
    std::ofstream fout;
    fout.open(dir + "\\results.txt", std::ios::out | std::ios::app);
    fout << para::algorithm_names[alg] << "\t";
    fout << features[data_index] << "\t";
    // fout << aco::EVAPORATE_COEF << "\t" << aco::ALPHA << "\t" << aco::BETA << "\t"; // 参数校准实验需要输出的，算法对比时可以关掉
    fout << result.str << "\n";
    fout.close();

    // ! 输出完整解
    // std::string name = dir + "\\answer_" + para::algorithm_names[alg] + "_" + std::to_string(data_index + 1) + ".txt";
    // fout.open(name);
    // fout << "distance\tspeed\theight\n";
    // int len = prob.getLengthDiscNum();
    // double dis, hei;
    // for (int i = 0; i < len; i++) {
    //     dis = resource::indexToLength(i, 0, resource::REF_UNIT_LENGTH);
    //     hei = resource::indexToHeight(optTraj.getHeightIndex(i), prob.getMinHeight(), resource::REF_UNIT_HEIGHT);
    //     fout << std::to_string(dis) << "\t";
    //     fout << std::to_string(speedSche[i]) << "\t";
    //     fout << std::to_string(hei) << "\n";
    // }
    // fout << " cost = " << std::to_string(result.cost) << "\n";
    // fout << "hcost = " << std::to_string(result.hcost) << "\n";
    // fout << "vcost = " << std::to_string(result.vcost) << "\n";
    // fout.close();
}

void solve_online(ProblemDisc2D &offprob, ProblemOnlineDisc2D &prob, para::Algorithm alg, std::string dir, int data_index) {
    std::clock_t sc = std::clock();
    Trajectory optTraj;
    // 求解
    online::ACOSolver_Online onlineSolver = online::ACOSolver_Online(&prob);
    std::vector<double> speedSche;
    onlineSolver.solve(speedSche);
    optTraj = onlineSolver.getTrajectory();
    std::clock_t ec = std::clock();
    // 记录结果
    Result result;
    result.hcost = onlineSolver.getHcost();
    result.vcost = onlineSolver.getVcost();
    // result.hcost = optTraj.calHeightCost();
    // result.vcost = energy_calculator::calSpeedCost(offprob, optTraj, speedSche);
    result.cost = result.hcost + result.vcost;
    result.runtime = (double) (ec - sc) / CLOCKS_PER_SEC;
    result.str = std::to_string(result.cost) + "\t" + std::to_string(result.hcost) + "\t" + std::to_string(result.vcost) + "\t" + std::to_string(result.runtime);
    results.push_back(result);
    
    // 保存结果（追加写入）
    std::ofstream fout;
    fout.open(dir + "\\results.txt", std::ios::out | std::ios::app);
    // fout.open(dir + "\\results-online.txt", std::ios::out | std::ios::app);
    // fout.open(dir + "\\A-temp-results-online.txt", std::ios::out | std::ios::app);
    fout << para::algorithm_names[alg] << "\t";
    fout << features[data_index] << "\t";
    fout << result.str << "\n";

    result.hcost = optTraj.calHeightCost(resource::HEIGHT_COST_PROPOR);
    result.vcost = energy_calculator::calSpeedCost(offprob, optTraj, speedSche);
    result.cost = result.hcost + result.vcost;
    result.str = std::to_string(result.cost) + "\t" + std::to_string(result.hcost) + "\t" + std::to_string(result.vcost) + "\t" + std::to_string(result.runtime);
    fout << "ACO-Online-2" << "\t";
    fout << features[data_index] << "\t";
    fout << result.str << "\n";

    fout.close();
}


/// @brief 用所有方法，求解所有测试数据
/// @param instance_num 
/// @param dir 
void solve_all_instance(int instance_num, std::string dir) {
    // std::vector<para::Algorithm> alg_set = {para::ACO, para::Greedy, para::PSO, para::GA, para::ACO_Online};
    // std::vector<para::Algorithm> alg_set = {para::DFS, para::ACO, para::PSO, para::GA, para::Greedy};
    // std::vector<para::Algorithm> alg_set = {para::DFS};
    std::vector<para::Algorithm> alg_set = {para::ACO, para::PSO, para::GA, para::Greedy, para::ACO_Online};

    std::string filename = "", feature = "";
    for (int i = 1; i <= instance_num; i++) {
        filename = filenames[i - 1];
        feature = features[i - 1];

        // 读入offline问题
        Problem2D probOff2D;
        probOff2D.initFromOnlineFile(filename);
        ProblemDisc2D probOffDisc2D = ProblemDisc2D(probOff2D);
        // 读入online问题
        ProblemOnline2D probOn2D;
        probOn2D.initFromOnlineFile(filename);
        ProblemOnlineDisc2D probOnDisc2D = ProblemOnlineDisc2D(probOn2D);
        
        for (para::Algorithm alg : alg_set) {

            if (alg == para::Algorithm::ACO_Online) {
                solve_online(probOffDisc2D, probOnDisc2D, alg, dir, i - 1);
            } else {
                solve(probOffDisc2D, alg, dir, i - 1);
            }
            
            std::cout << para::algorithm_names[alg] << ": " << i << "/" << instance_num << "\n";
        }
    }
}

void calibration_heuristic_component(int instance_num, std::string dir) {
    para::Algorithm alg;
    std::string filename = "", feature = "";

    // 打开支配筛选启发值
    alg = para::ACO_dominated;
    aco::HEURISTIC_FLAG = true;
    for (int i = 1; i <= instance_num; i++) {
        filename = filenames[i - 1];
        feature = features[i - 1];

        // 读入offline问题
        Problem2D prob2D;
        prob2D.initFromOnlineFile(filename);
        ProblemDisc2D probDisc2D = ProblemDisc2D(prob2D);

        solve(probDisc2D, alg, dir, i - 1);
        std::cout << para::algorithm_names[alg] << ": " << i << "/" << instance_num << "\n";
    }

    // 关闭筛选
    alg = para::ACO_normal;
    aco::HEURISTIC_FLAG = false;
    for (int i = 1; i <= instance_num; i++) {
        filename = filenames[i - 1];
        feature = features[i - 1];

        // 读入offline问题
        Problem2D prob2D;
        prob2D.initFromOnlineFile(filename);
        ProblemDisc2D probDisc2D = ProblemDisc2D(prob2D);

        solve(probDisc2D, alg, dir, i - 1);
        std::cout << para::algorithm_names[alg] << ": " << i << "/" << instance_num << "\n";
    }
}

void calibration_evaporation(int instance_num, std::string dir) {
    para::Algorithm alg = para::ACO;
    std::string filename = "", feature = "";

    for (double evp : para::evaporate_coefs) {

        aco::EVAPORATE_COEF = evp;
        std::cout << "evaporate = " << evp << "\n";

        for (int i = 1; i <= instance_num; i++) {
            filename = filenames[i - 1];
            feature = features[i - 1];

            // 读入offline问题
            Problem2D prob2D;
            prob2D.initFromOnlineFile(filename);
            ProblemDisc2D probDisc2D = ProblemDisc2D(prob2D);

            solve(probDisc2D, alg, dir, i - 1);
            std::cout << para::algorithm_names[alg] << ": " << i << "/" << instance_num << "\n";
        }
    }
}

void calibration_alpha_beta(int instance_num, std::string dir) {
    para::Algorithm alg = para::ACO;
    std::string filename = "", feature = "";

    for (double alpha : para::alphas) {
        for (double beta : para::betas) {
            aco::ALPHA = alpha;
            aco::BETA = beta;

            std::cout << "alpha = " << alpha << ", beta = " << beta << "\n";

            for (int i = 1; i <= instance_num; i++) {
                filename = filenames[i - 1];
                feature = features[i - 1];

                // 读入offline问题
                Problem2D prob2D;
                prob2D.initFromOnlineFile(filename);
                ProblemDisc2D probDisc2D = ProblemDisc2D(prob2D);

                solve(probDisc2D, alg, dir, i - 1);
                std::cout << para::algorithm_names[alg] << ": " << i << "/" << instance_num << "\n";
            }
        }
    }
}

/*
每次要改的地方：
1. para::sensor_nums
2. main里的direction
3. solve_all_instance()里的alg_set需不需要包含DFS
*/

int main() {

    std::srand((unsigned int) std::time(NULL));

    // 测试数据存储路径
    std::string direction = ".\\newexp\\5-10";
    // std::string direction = ".\\newexp\\test";

    // 生成数据
    generate_online_data(direction, true, 1);

    // 仿真实验
    int instance_num = 0;
    readInit(instance_num, direction, true);
    std::cout << "instance_number = " << instance_num << "\n\n";
    results.clear();
    solve_all_instance(instance_num, direction);

    // // calibration
    // int instance_num = 0;
    // readInit(instance_num, direction, true);
    // std::cout << "instance_number = " << instance_num << "\n\n";
    // results.clear();
    // // calibration_heuristic_component(instance_num, direction);
    // // calibration_alpha_beta(instance_num, direction);
    // calibration_evaporation(instance_num, direction);

    return 0;
}