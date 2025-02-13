import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator
from matplotlib.ticker import AutoMinorLocator

def power_consumption(v):
    p = 0.07 * v * v * v + 0.0391 * v * v - 13.196 * v + 390.95
    return p

if __name__ == '__main__':
    p = []
    v = []
    h = [] # h(v) = p(v) / v
    v_star = 13.98
    p_star = power_consumption(v_star)
    for _v in np.arange(0, 18, 0.01):
        v.append(_v)
        _p = power_consumption(_v)
        p.append(_p)
        h.append(_p / _v)
    fig, pv = plt.subplots()
    pv.plot(v, p, color='red')
    pv.set_ylabel(r'Power $(W)$', fontsize=14)
    pv.set_xlabel(r'Forward speed $(m/s)$', fontsize=14)
    plt.ylim(300,600)
    pv.spines['right'].set_visible(False)

    hv = pv.twinx()
    hv.plot(v, h, color='blue')
    hv.set_ylabel(r'Energy consumption to cover 1m (J/m)', fontsize=14)
    plt.ylim(20,50)
    plt.xlim(0,20)
    plt.show()
    