import matplotlib.pyplot as plt
import numpy as np

def pheromone_to_alpha(pheromone, dims):
    alpha_min = 0.003
    alphas = np.zeros(dims)
    ph_max = np.max(pheromone)
    ph_min = np.min(pheromone)
    ph_span = ph_max - ph_min
    print(ph_max, ph_min, ph_span)
    for i in range(dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                alphas[i][j][k] = alpha_min + (pheromone[i][j][k] - ph_min) / ph_span * (1 - alpha_min)
    # print(alphas)
    return alphas

def pheromone_to_color_depth(pheromone, dims):
    depth_min = 30
    depth_set = np.zeros(dims)
    ph_max = np.max(pheromone)
    ph_min = np.min(pheromone)
    ph_span = ph_max - ph_min
    print(ph_max, ph_min, ph_span)
    for i in range(dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                depth_set[i][j][k] = (pheromone[i][j][k] - ph_min) / ph_span
    return depth_set

def cal_rgb(color, base, rate):
    new_color = color * base + (color - color * base) * rate
    return new_color

if __name__ == '__main__':
    # 蚁群算法的信息素可视化
    path = "D:\\VSCodeWorkspace\\SSF-height\\newnewexp\\exp000\\pheromone_copy.txt"
    pheromone = []
    dims = []
    try:
        with open(file=path, mode="r") as file:
            buff = file.readline().strip()
            dims = [int(_) for _ in buff.split('\t')]
            for i in range(dims[0]):
                tmp = []
                for j in range(dims[1]):
                    buff = file.readline().strip()
                    tmp.append([float(_) for _ in buff.split('\t')])
                pheromone.append(tmp)
            pheromone = np.array(pheromone)
    except FileNotFoundError:
        print("File not found: " + path)
    # 输入的信息素维度
    print("dims =", dims)
    # 检查pheromone是否与输入大小一致
    print("shape =", pheromone.shape)

    depth_min = 0.1
    depth_set = pheromone_to_color_depth(pheromone, dims)
    for k in range(dims[2]):
        r = cal_rgb(255, depth_min, depth_set[0][0][k]) / 255.0
        g = cal_rgb(165, depth_min, depth_set[0][0][k]) / 255.0
        b = 0
        plt.plot([-1,0], [0,k], color=(r,g,b))
    for i in range(1,dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                r = cal_rgb(255, depth_min, depth_set[i][j][k]) / 255.0
                g = cal_rgb(165, depth_min, depth_set[i][j][k]) / 255.0
                b = 0
                plt.plot([i-1,i], [j,k], color=(r,g,b))
    # color = 'orange'
    # alphas = pheromone_to_alpha(pheromone, dims)
    # for k in range(dims[2]):
    #     plt.plot([-1,0], [0,k], color=color, alpha=alphas[0][0][k])
    # for i in range(1,dims[0]):
    #     for j in range(dims[1]):
    #         for k in range(dims[2]):
    #             plt.plot([i-1, i], [j, k], color=color, alpha=alphas[i][j][k])
    # print('prepare to save figure')
    # save_path = "D:\\Desktop\\ssf-journal\\figure\\pheromone visualization\\orange_3-2.pdf"
    # plt.savefig(save_path)
    # print('have saved figure: ' + save_path)
    plt.show()