import matplotlib.pyplot as plt
import numpy as np

def pheromone_to_alpha(pheromone, dims):
    alpha_min = 0.1
    alphas = np.zeros(dims)
    ph_max = np.max(pheromone)
    ph_min = np.min(pheromone)
    ph_span = ph_max - ph_min
    print(ph_max, ph_min, ph_span)
    for i in range(dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                alphas[i][j][k] = alpha_min + (pheromone[i][j][k] - ph_min) / ph_span * (1 - alpha_min)
    print(alphas)
    return alphas

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
    color = "orange"
    alphas = pheromone_to_alpha(pheromone, dims)
    for i in range(dims[0]):
        for j in range(dims[1]):
            for k in range(dims[2]):
                plt.plot([i, i+1], [j, k], color=color, alpha=alphas[i][j][k])
    plt.show()