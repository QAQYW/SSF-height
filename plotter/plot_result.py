import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    itemNumPerFile = 500
    # folders = ["5", "10", "20", "30", "40", "50"]
    folders = ["5", "10", "20"]
    values = []
    # 读所有result.txt
    for folder in folders:
        path = ".\\experiment\\" + folder + "\\result.txt"
        try:
            with open(file=path, mode='r') as file:
                print("read: " + path)
                line = file.readline()
                while line != "":
                    value = [e for e in line.strip().split() if e]
                    values.append(value)
                    line = file.readline()
        except FileNotFoundError:
            print("file not found: " + path)
    # ! 未写完