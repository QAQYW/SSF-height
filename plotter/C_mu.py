import matplotlib.pyplot as plt
import numpy as np
import math

def cal_C_mu(mu):
    t1 = mu * mu + mu + 1
    t2 = math.sqrt(t1)
    t3 = mu + 2 + 2 * t2
    t4 = 2 * t1 + 2 * mu + 2 + (2 * mu + 4) * t2
    c = math.sqrt(t3 * t3 * t3) / t4
    return c

# C_mu 与 mu 的关系

if __name__ == '__main__':
    mu = np.arange(0, 4, 0.01)
    c = [cal_C_mu(_) for _ in mu]
    plt.plot(mu, c)
    plt.show()