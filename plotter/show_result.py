import matplotlib.pyplot as plt

# 画散点图
def show(algNum, names, values, colors, markers, xLabel, xValues):
    x = [i for i in range(len(values[0]))]
    for i in range(algNum):
        plt.plot(x, values[i], c=colors[i], marker=markers[i], label=names[i])
    plt.xticks(x, xValues)
    plt.ticklabel_format(axis='y', style='sci', scilimits=(0,0))
    plt.legend()
    plt.ylabel('E')
    plt.xlabel(xLabel)
    plt.show()


if __name__ == '__main__':
    # filename = "" # 文件名
    # filepath = "" # 目录

    algNum = 4
    names = ["SSIF-GA", "SSIF-PSO", "SSIF-ACO-Online", "SSIF-ACO"]
    colors = [(254/255, 194/255, 17/255), (241/255, 64/255, 64/255), (51/255, 102/255, 153/255), (111/255, 184/255, 2/255)]
    
    # sensor num: n
    # aco     = [133190, 258100, 509881, 738112]
    # online  = [136889, 268715, 537841, 786522]
    # ga      = [221984, 444057, 908050, 1.35E+06]
    # pso     = [153665, 309428, 639115, 946566]
    # markers = ['o', 's', 'x', '+']
    # xLabel = r'$n$'
    # xValues = ['5', '10', '20', '30']

    # coef in height: C_{H,max}
    # aco     = [422255,413678,399336,404015]
    # online  = [436537,433817,425274,434338]
    # ga      = [735413,736577,724413,726452]
    # pso     = [488249,515833,519497,525195]
    # markers = ['o', 's', 'x', '+']
    # xLabel = r'$C_{H,max}$'
    # xValues = ['70', '100', '130', '160']

    # coef in width: C_{W,max}
    # aco     = [330156,386906,438010,484212]
    # online  = [343265,404548,463270,518884]
    # ga      = [646134,704151,758880,813691]
    # pso     = [434483,488227,540770,585294]
    # markers = ['o', 's', 'x', '+']
    # xLabel = r'$C_{W,max}$'
    # xValues = ['30', '40', '50', '60']
    
    # time prop: C_\tau
    # aco     = [166335,328096,491687,653166]
    # online  = [178172,346069,518625,687101]
    # ga      = [488973,650778,811189,971915]
    # pso     = [291881,440537,586338,730017]
    # markers = ['o', 's', 'x', '+']
    # xLabel = r'$C_\tau$'
    # xValues = ['0.5', '1.0', '1.5', '2.0']
    
    # swell coef: \mu
    aco     = [441755,420597,412421,389798,384534]
    online  = [465123,446422,434049,411429,405435]
    ga      = [759992,738896,729660,717121,707900]
    pso     = [542553,517249,510163,498716,492286]
    markers = ['o', 's', 'x', '+']
    xLabel = r'$\mu$'
    xValues = ['1.0', '1.5', '2.0', '2.5', '3.0']
    
    values = [ga, pso, online, aco]
    show(algNum, names, values, colors, markers, xLabel, xValues)