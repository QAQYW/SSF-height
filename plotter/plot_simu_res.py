import matplotlib.pyplot as plt
import numpy as np
# from matplotlib.ticker import MultipleLocator

# todo 改横坐标label

if __name__ == '__main__':
    markers=['o','s','^','v','x','D']
    colors=[(254/255, 194/255, 17/255), (241/255, 64/255, 64/255), (51/255, 102/255, 153/255), (111/255, 184/255, 2/255), (3/255, 140/255, 127/255), (140/255, 89/255, 59/255)]
    labels=['SSF-ACO','SSF-ACO-Online','SSF-GA','SSF-Only','SSF-PSO','SSF-SA']
    plt.tick_params(axis='x', labelsize=18)
    plt.tick_params(axis='y', labelsize=18)
    
    # # 传感器数量 n
    # # plt.subplot(2,4,1)
    # x=[5,10,15,20,25,30]
    # aco=[216893,388263,629461,832093,1007880,1167690]
    # online=[220292,389976,643520,834981,1023390,1189570]
    # ga=[242143,429976,702686,912663,1124680,1306370]
    # greedy=[227433,402224,654911,849598,1046990,1214120]
    # pso=[235854,417300,680610,884066,1089890,1265700]
    # sa=[243824,432666,707021,918044,1131240,1313090]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$n$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.title('Number of sensors.')
    # # plt.legend()
    # plt.show()
    
    # # 高度系数
    # # plt.subplot(2,4,2)
    # x=[40,60,80,100,120,140]
    # aco=[596060,585758,586605,587743,592868,597886]
    # online=[605066,589734,591965,596037,600861,607085]
    # ga=[666592,652486,652857,653839,654859,661865]
    # greedy=[611856,600849,602261,606798,609230,618085]
    # pso=[655192,637600,635449,634988,633004,639587]
    # sa=[670494,656277,656736,658117,659220,666158]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$C_{H,max}$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()

    # # 宽度系数
    # # plt.subplot(2,4,3)
    # x=[30,35,40,45,50,55,60]
    # aco=[470286,541867,579069,611726,635718,665488,771595]
    # online=[474153,542548,583217,627699,646110,673448,772853]
    # ga=[525837,598318,635538,682198,705300,736007,840250]
    # greedy=[478583,553038,588340,634468,658750,689259,791857]
    # pso=[503338,577039,613546,659968,683365,714320,818124]
    # sa=[530502,602709,639732,686584,709540,740510,844288]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$C_{W,max}$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()

    # # 传输时间系数
    # # plt.subplot(2,4,4)
    # x=[0.5,1,1.5,2,2.5,3]
    # aco=[156221,312676,436944,585220,718480,925369]
    # online=[157642,314022,443318,592136,730787,949980]
    # ga=[207825,363828,496563,644332,794786,1011340]
    # greedy=[160601,319477,449595,597695,748272,967049]
    # pso=[185982,343017,474458,622391,772242,990291]
    # sa=[211911,367924,500790,648783,798963,1015280]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$C_{T,max}$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()

    # # 膨胀系数
    # # plt.subplot(2,4,5)
    # x=[1,2,3,4,5,6]
    # aco=[705848,632632,585161,562575,544955,538346]
    # online=[707126,655172,592017,567000,549354,542487]
    # ga=[772602,709698,648706,619538,601780,593552]
    # greedy=[724034,662664,601348,574125,554992,549721]
    # pso=[750651,688365,626591,597916,578829,572593]
    # sa=[776858,713644,652861,623341,606310,597627]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$\mu_{max}$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()

    # # 无人机升降粒度
    # # ? 这个图的横坐标要不要把 1 和 3 也标出来
    # # plt.subplot(2,4,6)
    # x=[1,3,5,10,15,20,25,30]
    # aco=[607774,598763,597957,594485,594251,595647,602030,602612]
    # online=[612705,609415,608584,603866,608149,607890,608108,609233]
    # ga=[1012270,759660,709752,671315,657327,651075,648245,648819]
    # greedy=[624326,624326,624326,624326,624326,624326,624326,624326]
    # pso=[800366,688189,666671,649549,643192,640432,640366,640596]
    # sa=[1051270,771997,717745,675726,660287,653477,649294,646414]
    # y=[aco,online,ga,greedy,pso,sa]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$\delta$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()

    # 垂直能耗系数
    # plt.subplot(2,4,7)
    # x=[0,1,2,3,4,5,10,15,20,25,30,35,40]
    # aco=[612076,612640,613203,613767,614330,614894,617711,620529,623346,626164,627513,626705,627888]
    # online=[625716,626196,627337,626664,626965,626819,626470,627388,626256,627497,628981,631799,634616]
    # ga=[641275,681193,719693,758103,796759,835699,1027930,1221480,1413070,1607930,1797550,1995440,2185430]
    # greedy=[634837,634837,634837,634837,634837,634837,634837,634837,634837,634837,634837,634837,634837]
    # pso=[641052,659938,676235,693041,708823,725672,807285,890208,974329,1054200,1138180,1235440,1299140]
    # sa=[641092,685164,727997,770594,813448,856517,1068540,1280060,1496320,1709680,1916530,2132350,2340010]
    x=[0,5,10,15,20,25,30,35,40]
    aco=[612076,614894,617711,620529,623346,626164,627513,626705,627888]
    online=[625716,626819,626470,627388,626256,627497,628981,631799,634616]
    ga=[641275,835699,1027930,1221480,1413070,1607930,1797550,1995440,2185430]
    greedy=[634837,634837,634837,634837,634837,634837,634837,634837,634837]
    pso=[641052,725672,807285,890208,974329,1054200,1138180,1235440,1299140]
    sa=[641092,856517,1068540,1280060,1496320,1709680,1916530,2132350,2340010]
    y=[aco,online,ga,greedy,pso,sa]
    for i in range(len(y)):
        for j in range(len(y[i])):
            y[i][j] /= 100000
        plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    plt.xlabel(r'$q$', fontsize=24)
    plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # plt.legend()
    # plt.show()
    # plt.legend(bbox_to_anchor=(1.9,1), loc='upper right')

    # # plt.subplot(2,4,8)
    # # 524310
    handles, labels = plt.gca().get_legend_handles_labels()
    print(handles)
    legend = plt.legend(
        [handles[5],handles[2],handles[4],handles[3],handles[1],handles[0]], 
        [labels[5],labels[2],labels[4],labels[3],labels[1],labels[0]]
    )
    fig = plt.figure()
    fig.legend(handles=legend.legendHandles, loc='upper left')
    # plt.subplots_adjust(left=0,right=1,top=1,bottom=0) 
    # plt.subplots_adjust()
    # plt.tight_layout()
    plt.show()

    # # control communication range 半径比
    # x=[1,2,3,4,5,6,7,8,9,10]
    # aco=[603351,603351,603351,603351,603351,603351,603351,603351,603351,603351]
    # online=[619792,614693,606878,605556,604256,603693,603695,603522,603529,603529]
    # y=[aco,online]
    # for i in range(len(y)):
    #     for j in range(len(y[i])):
    #         y[i][j] /= 100000
    #     plt.plot(x,y[i],marker=markers[i],color=colors[i],label=labels[i])
    # plt.xlabel(r'$C_R$', fontsize=24)
    # plt.ylabel(r'$E\ (\times 10^5J)$', fontsize=24)
    # # plt.legend()
    # plt.show()
