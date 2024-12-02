import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':
    # 蚁群算法的信息素可视化
    path = "D:\\VSCodeWorkspace\\SSF-height\\newnewexp\\exp000\\pheromone_copy.txt"
    pheromone = []
    try:
        with open(file=path, mode="r") as file:
            buff = file.readline().strip()
            dims = [int(_) for _ in buff.split('\t')]
            # 输入的信息素维度
            print("dims =", dims)
            for i in range(dims[0]):
                tmp = []
                for j in range(dims[1]):
                    buff = file.readline().strip()
                    tmp.append([float(_) for _ in buff.split('\t')])
                pheromone.append(tmp)
    except FileNotFoundError:
        print("File not found: " + path)
    # 检查pheromone是否与输入大小一致
    print("shape =", np.array(pheromone).shape)
    