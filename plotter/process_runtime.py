import numpy as np
import matplotlib.pyplot as plt

def process(inpath, outpath, line_num, repeat_num):
    data = {}

    try:
        with open(inpath, 'r') as infile:
            line = infile.readline()
            print(line)
            for i in range(line_num):
                line = infile.readline().strip()
                print(line)
                temp = line.split('\t')
                alg = temp[0]             # algorithm name
                num = int(temp[1])        # sensor number
                runtime = float(temp[2])  # runtime
                flytime = float(temp[3])  # flght time
                if alg not in data:
                    data[alg] = []
                data[alg].append((num, runtime, flytime))
    except Exception as e:
        print(f'Func process(): read: {e}')
    
    avg_runtime = {}
    avg_flytime = {}
    alg_set = dict()
    num_set = dict()
    for alg in data:
        alg_set[alg] = 1
        for item in data[alg]:
            num, runtime, flytime = item
            num_set[num] = 1
            if (alg, num) not in avg_runtime:
                avg_runtime[(alg, num)] = 0
                avg_flytime[(alg, num)] = 0
            avg_runtime[(alg, num)] += runtime
            avg_flytime[(alg, num)] += flytime
    for alg in alg_set:
        for num in num_set:
            avg_runtime[(alg, num)] /= repeat_num
            avg_flytime[(alg, num)] /= repeat_num
    
    try:
        with open(outpath, 'w') as outfile:

            outfile.write('runtime\n')
            print('runtime')
            temp = ''
            for num in num_set:
                temp += '\t' + str(num)
            outfile.write(temp + '\n')
            print(temp)
            for alg in alg_set:
                temp = alg
                for num in num_set:
                    temp += '\t' + str(avg_runtime[(alg, num)])
                outfile.write(temp + '\n')
                print(temp)

            outfile.write('flytime\n')
            print('flytime')
            temp = ''
            for num in num_set:
                temp += '\t' + str(num)
            outfile.write(temp + '\n')
            print(temp)
            for alg in alg_set:
                temp = alg
                for num in num_set:
                    temp += '\t' + str(avg_flytime[(alg, num)])
                outfile.write(temp + '\n')
                print(temp)
    except Exception as e:
        print(f'Func process(): write: {e}')
    
    return data, avg_runtime, avg_flytime

if __name__ == '__main__':
    line_num = 720
    repeat_num = 10
    
    data, avg_runtime, avg_flytime = process('./plotter/runtime-origin.txt', './plotter/runtime-processed.txt', line_num, repeat_num)

    # draw