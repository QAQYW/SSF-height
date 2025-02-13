import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator
from matplotlib.ticker import AutoMinorLocator

def power_consumption(v):
    p = 0.07 * v * v * v + 0.0391 * v * v - 13.196 * v + 390.95
    return p

if __name__ == '__main__':
    # # - p(v) 曲线的图
    # # todo 标记出 v_star 等点，还有v_m, v_{max}
    # p = []
    # v = []
    v_star = 13.98
    p_star = power_consumption(v_star)
    # for i in np.arange(0,18,0.01):
    #     v.append(i)
    #     p.append(power_consumption(i))
    #     # if power_consumption(i) < power_consumption(v_star):
    #     #     v_star = i
    # # print('v* = ', v_star)
    # fig, ax = plt.subplots()
    # plt.plot(v, p, color='red', zorder=1)
    # plt.ylim(300,600)
    # plt.xlim(0,20)
    # # plt.grid() # 网格线
    # plt.tick_params(axis='x', labelsize=14)
    # plt.tick_params(axis='y', labelsize=14)
    # plt.xlabel(r'Forward speed $(m/s)$', fontsize=14)
    # plt.ylabel(r'Power $(W)$', fontsize=14)
    # plt.xticks(np.arange(0,21,5))
    # # plt.text(v[100], p[100]+50, r'$p(v)=0.07v^3+0.0391v^2-13.196v+390.95$', fontsize=14)
    # # plt.text(v_star+0.5, power_consumption(v_star), r'$v^*$: the most energy-efficient forward speed', fontsize=14)
    # plt.scatter([v_star], [p_star], color='black', s=25, marker='o', zorder=2)
    # # plt.plot([v_star, v_star], [0, p_star], linestyle='--', color='black')
    # # plt.text(v_star-0.5, 280, r'$v^*$', fontsize=14)
    # # 在曲线附近添加标签，并用箭头指向点 (3, 9)
    # # ax.annotate('My Label', xy=(3, 9), xytext=(4, 15),
    # #         arrowprops=dict(facecolor='black', shrink=0.05),
    # #         fontsize=12)
    # ax.annotate('The energy-efficient speed: '+ r'$v^*$',
    #             xy=(v_star-0.05, p_star+1),
    #             xytext=(v_star-13, p_star+50),
    #             fontsize=14,
    #             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8))
    # # plt.gca().yaxis.set_major_locator(MultipleLocator(100))
    # plt.show()


    # - 画 proof of lemma 2 的图

    x0 = 13.98
    y0 = power_consumption(x0)
    # 竖线 x=12
    plt.plot([x0, x0], [0, y0], color='black')
    # 横线 y=0
    plt.plot([0, x0], [0, 0], color='black')
    # 斜线（三角形斜边）
    plt.plot([0,x0], [0,y0], color='black') # (135/255,206/255,250/255)

    # 曲线
    v = []
    p = []
    # h = []
    for i in np.arange(0,18,0.01):
        v.append(i)
        pp = power_consumption(i)
        p.append(pp)
        # h.append(pp/i)
    plt.plot(v, p, color='red', label=r'$p(v)$')
    plt.gca().xaxis.set_major_locator(MultipleLocator(5))
    plt.gca().yaxis.set_major_locator(MultipleLocator(100))
    plt.gca().xaxis.set_minor_locator(AutoMinorLocator(5))
    plt.gca().yaxis.set_minor_locator(AutoMinorLocator(5))
    plt.grid(True, which='both', axis='both', linestyle='-', color=(211/255,211/255,211/255)) # 浅灰色
    plt.grid(True, which='minor', axis='both', linestyle='-', color=(211/255,211/255,211/255)) # 浅灰色
    plt.xlim(0,20.01)
    plt.ylim(0,600)
    plt.tick_params(axis='x', labelsize=18)
    plt.tick_params(axis='y', labelsize=18)
    plt.xlabel(r'Forward speed $(m/s)$', fontsize=18)
    plt.ylabel(r'Power $(W)$', fontsize=18)
    plt.gca().spines['top'].set_visible(False)
    plt.gca().spines['right'].set_visible(False)

    # 切线（虚线）
    slope = (power_consumption(x0+0.01) - power_consumption(x0-0.01)) / 0.02
    plt.plot([x0-7,x0+5], [y0-slope*7,y0+slope*5], color=(135/255,206/255,250/255), linestyle='--')

    plt.text(x0+0.25, y0/2, r'$p(v)$', fontsize=18)
    plt.text(x0/2, 5, r'$v$', fontsize=18)
    plt.text(5, y0/x0*5-30, r'$p(v)/v$', fontsize=18)
    plt.text(x0+3, y0+slope*2, r"$p'(v)$", fontsize=18)

    plt.scatter([v_star], [p_star], marker='*', s=150, zorder=2)
    plt.annotate(r'$v^*$',
                xy=(v_star-0.05, p_star+1),
                xytext=(v_star-3, p_star+50),
                fontsize=18,
                arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8))

    plt.show()