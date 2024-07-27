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
#include "sa.h"
#include "greedy.h"
#include "greedy2.h"
#include "greedy3.h"
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
    const int sensor_nums[] = {5, 10, 15, 20, 25, 30};

    // 水滴曲线最大高度（米），参考值 {70, 100, 130, 160}
    const double max_y_mults[] = {60, 80, 100, 120, 140, 160};

    // 水滴曲线最大宽度（米），参考值 {30, 40, 50, 60}
    const double max_x_milts[] = {30, 35, 40, 45, 50, 55, 60};

    // // 水滴最宽处宽度与路径总长度线性相关的系数，参考值 {0.2, 0.3, 0.4, 0.5, 0.6}
    // const double max_x_mult_coefs[] = {0.2, 0.3, 0.4, 0.5, 0.6};

    // // 传输时间与传输范围大小的平方相关的系数，参考值 {0.05, 0.1, 0.2, 0.3}
    // const double max_time_range_props[] = {0.05, 0.1, 0.2, 0.3};

    // 传输时间与传输范围大小线性相关的系数，参考值 {0.5, 1, 1.5, 2}
    const double time_props[] = {0.5, 1, 1.5, 2, 2.5, 3};

    // 最大膨胀系数，参考值 {1, 1.5, 2, 2.5, 3}
    const double max_swells[] = {1, 1.5, 2, 2.5, 3, 3.5};

    // 高度变化粒度
    const double d_heights[] = {1, 3, 5, 10, 15, 20, 25, 30};

    // 高度变化的能耗系数
    const double height_cost_propors[] = {0, 1, 2, 3, 4, 5, 10, 15, 20, 25, 30, 35, 40};

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
    const std::string algorithm_names[11] = {
        "DFS",
        "ACO",
        "PSO",
        "GA",
        "SA",
        "Greedy",
        "Greedy2",
        "Greedy3",
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
        Greedy = 5,
        Greedy2 = 6,
        Greedy3 = 7,
        ACO_Online = 8,
        ACO_dominated = 9,  // for calibration
        ACO_normal = 10     // for calibration
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
    Result result;
    std::vector<double> speedSche;
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
    } else if (alg == para::Algorithm::SA) {
        // 4
        sa::SASolver saSolver = sa::SASolver(&prob);
        saSolver.solve();
        optTraj = saSolver.getTrajectory();
    } else if (alg == para::Algorithm::Greedy) {
        // 5
        greedy::GreedySolver greedySolver = greedy::GreedySolver(&prob);
        greedySolver.solve();
        optTraj = greedySolver.getTrajectory();
    } else if (alg == para::Algorithm::Greedy2) {
        // 6
        greedy2::GreedySolver2 greedySolver2 = greedy2::GreedySolver2(&prob);
        greedySolver2.solve();
        optTraj = greedySolver2.getTrajectory();
    } else if (alg == para::Algorithm::Greedy3) {
        // 7
        greedy3::GreedySolver3 greedySovler3 = greedy3::GreedySolver3(&prob);
        greedySovler3.solve();
        optTraj = greedySovler3.getTrajectory();
        result.hcost = greedySovler3.getHcost();
        result.vcost = greedySovler3.getVcost();
        result.cost = greedySovler3.getCost();
        speedSche = greedySovler3.getSpeedSche();
    }
    std::clock_t ec = std::clock();
    // 记录结果
    if (alg != para::Algorithm::Greedy3) {
        result.hcost = optTraj.calHeightCost(resource::HEIGHT_COST_PROPOR);
        result.vcost = energy_calculator::calSpeedCost(prob, optTraj, speedSche);
        result.cost = result.hcost + result.vcost;
    }
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
    // std::vector<para::Algorithm> alg_set = {para::ACO};
    std::vector<para::Algorithm> alg_set = {/*para::ACO,*/ para::PSO, para::GA, para::SA, para::Greedy, para::Greedy2, para::Greedy3, para::ACO_Online};

    std::string filename = "", feature = "";
    for (int i = 1; i <= instance_num; i++) {
        filename = filenames[i - 1];
        feature = features[i - 1];

        std::vector<std::string> parameters;
        tools::splitString(parameters, feature, '\t');
        double hcost_propor = std::stod(parameters[7]);
        // std::cout << "propor = " << hcost_propor << "\n";
        resource::HEIGHT_COST_PROPOR = hcost_propor;

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

// sensor_num, max_y_mutls, max_x_mults, time_prop, max_swell
void run_exp(std::string dir, int &count_data) {
    /*
        sensor_num: 传感器数量          1-6
        max_y_mutl: 传输范围高度       7-12
        max_x_mult: 传输范围宽度       13-19
        time_prop: 传输时间系数/数据量  20-25
        max_swell: 膨胀系数             26-31
        d_height: 高度变化粒度
        hcost_propor: 高度变化能耗系数
    */
    int sensor_num = 15;
    double max_x_mult = 45;
    double max_y_mult = 120;
    double time_prop = 2;
    double max_swell = 2.5;
    double d_height = 10;
    double hcost_propor = 1;

    filenames.clear();
    features.clear();
    std::vector<unsigned int> seeds;

    // // sensor_num
    // for (int sensor_num : para::sensor_nums) {
    //     ++count_data;
    //     DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    //     unsigned int seed = std::rand();
    //     seeds.push_back(seed);
    //     dg2.generate_save_online(seed, count_data);
        
    //     // 测试数据特征值
    //     std::string feature = std::to_string(count_data) + "\t"
    //         + std::to_string(sensor_num) + "\t"
    //         + std::to_string(max_y_mult) + "\t"
    //         + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
    //         + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
    //         + std::to_string(max_swell) + "\t"
    //         + std::to_string(d_height) + "\t"
    //         + std::to_string(hcost_propor) + "\t"
    //         + "sensor_num";
    //     features.push_back(feature);
    //     std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
    //     filenames.push_back(filename);
    // }

    // // max_y_mult
    // for (double max_y_mult : para::max_y_mults) {
    //     ++count_data;
    //     DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    //     unsigned int seed = std::rand();
    //     seeds.push_back(seed);
    //     dg2.generate_save_online(seed, count_data);
        
    //     // 测试数据特征值
    //     std::string feature = std::to_string(count_data) + "\t"
    //         + std::to_string(sensor_num) + "\t"
    //         + std::to_string(max_y_mult) + "\t"
    //         + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
    //         + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
    //         + std::to_string(max_swell) + "\t"
    //         + std::to_string(d_height) + "\t"
    //         + std::to_string(hcost_propor) + "\t"
    //         + "max_y_mult";
    //     features.push_back(feature);
    //     std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
    //     filenames.push_back(filename);
    // }

    // // max_x_mult
    // for (double max_x_mult : para::max_x_milts) {
    //     ++count_data;
    //     DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    //     unsigned int seed = std::rand();
    //     seeds.push_back(seed);
    //     dg2.generate_save_online(seed, count_data);
        
    //     // 测试数据特征值
    //     std::string feature = std::to_string(count_data) + "\t"
    //         + std::to_string(sensor_num) + "\t"
    //         + std::to_string(max_y_mult) + "\t"
    //         + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
    //         + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
    //         + std::to_string(max_swell) + "\t"
    //         + std::to_string(d_height) + "\t"
    //         + std::to_string(hcost_propor) + "\t"
    //         + "max_x_mult";
    //     features.push_back(feature);
    //     std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
    //     filenames.push_back(filename);
    // }

    // // time_prop
    // for (double time_prop : para::time_props) {
    //     ++count_data;
    //     DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    //     unsigned int seed = std::rand();
    //     seeds.push_back(seed);
    //     dg2.generate_save_online(seed, count_data);
        
    //     // 测试数据特征值
    //     std::string feature = std::to_string(count_data) + "\t"
    //         + std::to_string(sensor_num) + "\t"
    //         + std::to_string(max_y_mult) + "\t"
    //         + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
    //         + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
    //         + std::to_string(max_swell) + "\t"
    //         + std::to_string(d_height) + "\t"
    //         + std::to_string(hcost_propor) + "\t"
    //         + "time_prop";
    //     features.push_back(feature);
    //     std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
    //     filenames.push_back(filename);
    // }

    // max_swell
    for (double max_swell : para::max_swells) {
        ++count_data;
        DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
        unsigned int seed = std::rand();
        seeds.push_back(seed);
        dg2.generate_save_online(seed, count_data);
        
        // 测试数据特征值
        std::string feature = std::to_string(count_data) + "\t"
            + std::to_string(sensor_num) + "\t"
            + std::to_string(max_y_mult) + "\t"
            + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
            + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
            + std::to_string(max_swell) + "\t"
            + std::to_string(d_height) + "\t"
            + std::to_string(hcost_propor) + "\t"
            + "max_swell";
        features.push_back(feature);
        std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
        filenames.push_back(filename);
    }

    int tot = features.size();

    std::ofstream fout;
    fout.open(dir + "\\features.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) {
        fout << features[i] << "\n";
    }
    fout.close();
    fout.open(dir + "\\online_filename_set.txt", std::ios::out | std::ios::app);
    // fout << std::to_string(count_data) << "\n";
    for (int i = 0; i < tot; i++) {
        fout << filenames[i] << "\n";
    }
    fout.close();
}

// d_height, hcost_coef
void run_exp2(std::string dir, int &count_data) {
    /*
        d_height: 1-6
        hcost_propor: 7-23
    */

    int sensor_num = 15;
    double max_x_mult = 45;
    double max_y_mult = 120;
    double time_prop = 2;
    double max_swell = 2.5;
    double d_height = 10;
    double hcost_propor = 1;

    filenames.clear();
    features.clear();
    std::vector<unsigned int> seeds;

    // d_height
    for (double d_height : para::d_heights) {
        ++count_data;
        DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
        unsigned int seed = std::rand();
        seeds.push_back(seed);
        dg2.generate_save_online(seed, count_data);

        // 测试数据特征值
        std::string feature = std::to_string(count_data) + "\t"
            + std::to_string(sensor_num) + "\t"
            + std::to_string(max_y_mult) + "\t"
            + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
            + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
            + std::to_string(max_swell) + "\t"
            + std::to_string(d_height) + "\t"
            + std::to_string(hcost_propor) + "\t"
            + "d_height";
        features.push_back(feature);
        std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
        filenames.push_back(filename);
    }

    // hcost_propor
    for (double hcost_propor : para::height_cost_propors) {
        ++count_data;
        DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
        unsigned int seed = std::rand();
        seeds.push_back(seed);
        dg2.generate_save_online(seed, count_data);
        
        // 测试数据特征值
        std::string feature = std::to_string(count_data) + "\t"
            + std::to_string(sensor_num) + "\t"
            + std::to_string(max_y_mult) + "\t"
            + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
            + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
            + std::to_string(max_swell) + "\t"
            + std::to_string(d_height) + "\t"
            + std::to_string(hcost_propor) + "\t"
            + "hcost_propor";
        features.push_back(feature);
        std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(count_data) + ".txt";
        filenames.push_back(filename);
    }

    int tot = features.size();

    std::ofstream fout;
    fout.open(dir + "\\features.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) fout << features[i] << "\n";
    fout.close();

    fout.open(dir + "\\online_filename_set.txt", std::ios::out | std::ios::app);
    // fout << std::to_string(count_data) << "\n";
    for (int i = 0; i < tot; i++) fout << filenames[i] << "\n";
    fout.close();
}

/// @brief for var 'd_height'
/// @param dir 
/// @param count_data 
void run_exp3(std::string dir, int &count_data) {

    int sensor_num = 15;
    double max_x_mult = 45;
    double max_y_mult = 120;
    double time_prop = 2;
    double max_swell = 2.5;
    double d_height = 10;
    double hcost_propor = 1;

    filenames.clear();
    features.clear();
    std::vector<unsigned int> seeds;

    std::vector<double> d_height_set;
    for (double dh : para::d_heights)
        d_height_set.push_back(dh);
    
    int st = count_data + 1;
    int ed = count_data + d_height_set.size();

    DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    unsigned int seed = std::rand();
    for (int i = st; i <= ed; i++) {
        seeds.push_back(seed);
        std::string feature = std::to_string(i) + "\t"
            + std::to_string(sensor_num) + "\t"
            + std::to_string(max_y_mult) + "\t"
            + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
            + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
            + std::to_string(max_swell) + "\t"
            + std::to_string(d_height_set[i - st]) + "\t"
            + std::to_string(hcost_propor) + "\t"
            + "d_height";
        features.push_back(feature);
        std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }
    dg2.generate_save_online_for_dheight(seed, count_data, d_height_set);

    int tot = features.size();

    std::ofstream fout;
    fout.open(dir + "\\features.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) fout << features[i] << "\n";
    fout.close();

    fout.open(dir + "\\online_filename_set.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) fout << filenames[i] << "\n";
    fout.close();
}

/// @brief for var 'height_cost_propor'
/// @param dir 
/// @param count_data 
void run_exp4(std::string dir, int &count_data) {

    int sensor_num = 15;
    double max_x_mult = 45;
    double max_y_mult = 120;
    double time_prop = 2;
    double max_swell = 2.5;
    double d_height = 10;
    double hcost_propor = 1;

    filenames.clear();
    features.clear();
    std::vector<unsigned int> seeds;
    
    std::vector<double> hcost_propor_set;
    for (double propor : para::height_cost_propors)
        hcost_propor_set.push_back(propor);
    
    int st = count_data + 1;
    int ed = count_data + hcost_propor_set.size();

    DataGenerator2 dg2 = DataGenerator2(dir, sensor_num, max_y_mult, max_x_mult, time_prop, max_swell, d_height);
    unsigned int seed = std::rand();
    for (int i = st; i <= ed; i++) {
        seeds.push_back(seed);
        std::string feature = std::to_string(i) + "\t"
            + std::to_string(sensor_num) + "\t"
            + std::to_string(max_y_mult) + "\t"
            + std::to_string(max_x_mult) + "\t" // + std::to_string(max_x_mult_coef) + "\t"
            + std::to_string(time_prop) + "\t" // + std::to_string(max_time_range_prop) + "\t"
            + std::to_string(max_swell) + "\t"
            + std::to_string(d_height) + "\t"
            + std::to_string(para::height_cost_propors[i - st]) + "\t"
            + "hcost_propor";
        features.push_back(feature);
        std::string filename = dir + "\\" + dg2.filenameBaseOnline + std::to_string(i) + ".txt";
        filenames.push_back(filename);
    }
    dg2.generate_save_online_for_hcostprop(seed, count_data, hcost_propor_set);

    int tot = features.size();

    std::ofstream fout;
    fout.open(dir + "\\features.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) fout << features[i] << "\n";
    fout.close();

    fout.open(dir + "\\online_filename_set.txt", std::ios::out | std::ios::app);
    for (int i = 0; i < tot; i++) fout << filenames[i] << "\n";
    fout.close();
}

/*
    1. para::sensor_nums
    2. main里的direction
    3. alg_set
*/

int main() {

    // std::srand((unsigned int) std::time(NULL));

    // // 是否筛选
    // aco::HEURISTIC_FLAG = true;

    // // 测试数据存储路径
    // // std::string direction = ".\\newexp\\test";
    // // std::string direction = ".\\newexp\\newexp";
    // std::string direction = ".\\newexp\\newexp2";

    // // 生成数据
    // // generate_online_data(direction, true, 1);
    // // run_exp(direction);
    // run_exp2(direction);

    // // 仿真实验
    // int instance_num = 0;
    // readInit(instance_num, direction, true);
    // std::cout << "instance_number = " << instance_num << "\n\n";
    // results.clear();
    // solve_all_instance(instance_num, direction);

    // // // calibration
    // // int instance_num = 0;
    // // readInit(instance_num, direction, true);
    // // std::cout << "instance_number = " << instance_num << "\n\n";
    // // results.clear();
    // // // calibration_heuristic_component(instance_num, direction);
    // // // calibration_alpha_beta(instance_num, direction);
    // // calibration_evaporation(instance_num, direction);



    /* -------------------------------------------------------------------------- */
    /*                                   new exp                                  */
    /* -------------------------------------------------------------------------- */

    std::srand((unsigned int) std::time(NULL));
    aco::HEURISTIC_FLAG = true;

    int repeat = 80;
    int count = 0;
    int count1 = 0, count2 = 0, count3 = 0, count4 = 0;
    std::string direction = "";
    for (int i = 0; i < repeat; i++) {
        // direction = ".\\newnewexp\\exp1"; // path
        // run_exp(direction, count1);

        // direction = ".\\newnewexp\\exp2"; // path
        // run_exp2(direction, count2);

        // direction = ".\\newnewexp\\exp3"; // path
        // run_exp3(direction, count3);

        // direction = ".\\newnewexp\\exp4"; // path
        // run_exp4(direction, count4);

        direction = ".\\newnewexp\\exp5";   // ! 修改路径(文件夹exp__)
        run_exp(direction, count);          // ! 修改run_exp()函数，修改count变量
        // ! 修改online_filename_set.txt
    }

    features.clear();
    filenames.clear();
    int instance_num = 0;
    direction = ".\\newnewexp\\exp5";       // ! 修改路径(文件夹exp__)
    readInit(instance_num, direction, true);
    std::cout << "instance num = " << instance_num << "\n\n";
    results.clear();
    solve_all_instance(instance_num, direction);

    return 0;
}