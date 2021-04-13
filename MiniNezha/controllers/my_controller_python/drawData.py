# -*- coding: UTF-8 -*-
import csv
from matplotlib import pyplot as plt


class drawer:
    def __init__(self, height=0):
        self.bodyHeight = []
        self.opt_vel = 0
        self.obj_val = 0
        self.a, self.b, self.c, self.d = 0, 0, 0, 0
        self.height = height
        self.line = 0

        self.isPointPos = False
        self.fileName = ''
        self.txtFileName = ''

    def changeArgs(self, height, line):
        self.height = height
        self.line = line
        if height == 0.2:
            csvName = 'args_9.953-h=0.2.csv'
        elif height == 0.3:
            csvName = 'args_14.767-h=0.3.csv'
        elif height == 0.4:
            csvName = 'args_20.788-h=0.4.csv'
        else:
            csvName = ''
        with open(csvName, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for i, rows in enumerate(reader):
                if i == line:
                    row = rows
        self.opt_vel, self.obj_val, self.d, self.c, self.b, self.a = row[0], row[1], row[2], row[3], row[4], row[5]
        # print(self.opt_vel, self.obj_val, self.d, self.c, self.b, self.a)
        file_handle = open('args.txt', mode='w')
        file_handle.writelines(['{\'opt_vel\'', ': ', str(self.opt_vel), ', ',
                                '\'objective_value\': ', str(self.obj_val), ', ',
                                '\'jump_d\': ', str(self.d), ', ',
                                '\'jump_c\': ', str(self.c), ', ',
                                '\'jump_b\': ', str(self.b), ', ',
                                '\'jump_a\': ', str(self.a), '}'])
        file_handle.close()

    def drawData(self):
        with open(self.txtFileName) as f:
            reader = csv.reader(f)
            header_row = next(reader)
            count = 0
            times, wheelHeights, wheelLengths, torques = [], [], [], []
            timePs, wheelHeightPs, wheelLengthPs, torquePs = [], [], [], []
            for row in reader:

                time = float(row[0])
                times.append(time)

                wheelHeight = float(row[1])
                wheelHeights.append(wheelHeight)

                wheelLength = float(row[2])
                wheelLengths.append(wheelLength)

                torque = float(row[3])
                torques.append(torque)

                if count % 100 == 0:
                    timePs.append(time)
                    wheelHeightPs.append(wheelHeight)
                    wheelLengthPs.append(wheelLength)
                    torquePs.append(torque)

                count += 1
        # 根据数据绘制图形
        fig = plt.figure(dpi=128, figsize=(10, 6))
        # plt.plot(wheelLengths, wheelHeights, c='red')
        # plt.scatter(wheelLengthPs, wheelHeightPs, marker = 'o',c='red')
        # type = "Wheel trajectory with distance"
        # title = type + str(self.height) + '_' + str(self.)
        # xlabel = 'Distance'
        # ylabel = 'Wheel Height'

        plt.plot(times, torques, c='red')
        type = "Knee torque vs time"
        xlabel = 'Time'
        ylabel = 'Knee Torque'

        # 设置图形格式
        title = type + str(self.height) + '_' + str(self.line)
        plt.title(title, fontsize=24)
        plt.xlabel(xlabel, fontsize=16)
        fig.autofmt_xdate()  # 绘制斜的日期标签，以免它们彼此重叠
        plt.ylabel(ylabel, fontsize=16)
        plt.tick_params(axis='both', which='major', labelsize=16)
        plt.grid()

        # plt.show()
        figureName = "./chart/" + self.fileName + '_' + type + ".jpg"
        plt.savefig(figureName, bbox_inches='tight')
