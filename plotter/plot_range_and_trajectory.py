from math import sqrt
import matplotlib.pyplot as plt
import numpy as np

vStar = 13.98
# vStar = 5

def draw_single_sensor(sensor, length, id):
    height_ulp = 1
    mid = sensor[0]
    xCoef = sensor[1]
    yCoef = sensor[2]
    swell = sensor[3]
    yMax = 2.0 / yCoef
    
    xl = []; yl = []
    xr = []; yr = []
    for h in np.arange(height_ulp, yMax, height_ulp):
        # 计算高度h下的横坐标
        yBar = yCoef * h
        p = 2 * yBar * (yBar - 1) + swell
        q = yBar * yBar * yBar * (yBar - 2)
        temp1 = p * p - 4 * q
        temp2 = (sqrt(temp1) - p) / 2
        r = sqrt(temp2) / xCoef
        # left = max(0, mid - r)       # 左端点
        # right = min(length, mid + r) # 右端点
        left  = mid - r # 左端点
        right = mid + r # 右端点
        # 记录横纵坐标
        xl.append(left)
        yl.append(h)
        xr.append(right)
        yr.append(h)
    x = [mid] + xl + [mid] + xr[::-1] + [mid]
    y = [0] + yl + [yMax] + yr[::-1] + [0]
    
    # 绘制曲线
    plt.plot(x, y, linestyle="-", linewidth=1, label="S"+str(id+1))
    # marker="o", markersize=1, 


def draw_sensors(sensors, length):
    plt.figure()
    for id, sensor in enumerate(sensors):
        draw_single_sensor(sensor, length, id)


def draw_solution(solution, length):
    """
        飞行速度越慢 轨迹的颜色越深
    """
    # 水平
    for segment in solution:
        d = segment[0]
        v = segment[1]
        h = segment[2]
        plt.plot([d, d+1], [h, h], linewidth=2, color=plt.get_cmap("viridis")(v / vStar))
    # 垂直升降
    for i in range(int(length) - 1):
        seg1 = solution[i]
        seg2 = solution[i + 1]
        h1 = seg1[2]
        h2 = seg2[2]
        if h1 != h2:
            d = seg2[0]
            plt.plot([d, d], [h1, h2], linewidth=2, color='#000000') # 升降的轨迹统一用黑色


if __name__ == "__main__":
    # print("Hello world!")
    # direction = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\"
    # sensor_filename = "online_shape_1.txt"
    # solution_filename = "online_answer_aco_prop10_1.txt"

    dataIndex = "1"
    
    length = 859  #50
    minHeight = 0
    maxHeight = 0
    sensorNum = 0

    timestr = ""

    try:
        # 读取最新样例的timestr（文件夹名）
        filepath = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\timestr.txt"
        with open(file=filepath, mode="r") as file:
            timestr = file.readline().strip()
    except FileNotFoundError:
        print("File not found: " + filepath)
    
    # timestr = "2024_4_11_22_31_31"
    # timestr = "1900_1_1_0_0_1"
    print("\ntimestr = " + timestr + "\n")

    try:
        # 绘制传感器传输范围
        filepath = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\" + timestr + "\\online_shape_" + dataIndex + ".txt"
        with open(file=filepath, mode="r") as file:
            length    = float(file.readline().strip())  # 路径长度
            minHeight = float(file.readline().strip())  # 最小高度
            maxHeight = float(file.readline().strip())  # 最大高度
            sensorNum = int(file.readline().strip())    # 传感器数量
            sensors = [] # 传感器形状参数
            for i in range(sensorNum):
                # 每一行是一个传感器
                line = file.readline().strip()
                sensor = [float(x) for x in line.split()]
                print(sensor)
                sensors.append(sensor)
            draw_sensors(sensors, length)
    except FileNotFoundError:
        print("File not found: " + filepath)

    try:
        # 绘制飞行轨迹，用颜色深浅表示速度
        # 离线-暴力
        filepath = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\" + timestr + "\\offline_answer_naive_prop39_" + dataIndex + ".txt"
        # 离线-蚁群
        filepath = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\" + timestr + "\\offline_answer_aco_prop39_" + dataIndex + ".txt"
        # 在线-蚁群
        # filepath = "D:\\VSCodeWorkspace\\SSF-height\\tiny_test\\" + timestr + "\\online_answer_aco_prop39_" + dataIndex + ".txt"
        with open(file=filepath, mode="r") as file:
            file.readline() # 跳过第一行的标题
            solution = [] # 保存解（距离、速度、高度）
            for i in range(0, int(length)):
                line = file.readline().strip()
                segment = [float(x) for x in line.split()]
                solution.append(segment)
            draw_solution(solution, int(length))
            print(file.readline())
            print(file.readline())
            print(file.readline())
    except FileNotFoundError:
        print("File not found: " + filepath)
    
    # 颜色条
    cm = plt.get_cmap("viridis")
    sm = plt.cm.ScalarMappable(cmap=cm)
    sm.set_array(np.linspace(0, 14, 140))
    plt.colorbar(sm)

    # 图例（传感器）
    plt.legend()
    plt.xticks(range(0, int(length)))
    plt.grid()

    # axvline()是垂直直线，axhline()是水平直线
    plt.axvline(x=0, linestyle="--")
    plt.axvline(x=length, linestyle="--")

    # 显示图形
    plt.title("shape of sensors")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()