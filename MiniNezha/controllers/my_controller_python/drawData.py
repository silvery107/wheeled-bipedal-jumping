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

        self.takeOffIndex = 0
        self.startIndex = 0
        self.startTorqueIndex = 0
        self.highestIndex = 0

    def changeArgs(self, height, line, csvName=None):
        self.height = height
        self.line = line
        if csvName is None:
            if height == 0.2:
                csvName = 'args_9.953-h=0.2.csv'
            elif height == 0.3:
                csvName = 'args_14.767-h=0.3.csv'
            elif height == 0.4:
                csvName = 'args_20.788-h=0.4.csv'
            else:
                csvName = ''
        if csvName != '':
            with open(csvName, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for i, rows in enumerate(reader):
                    if i == line - 1:
                        row = rows
            self.opt_vel, self.obj_val, self.d, self.c, self.b, self.a = row["opt_vel"], row["objective_value"], row[
                "jump_d"], row["jump_c"], row["jump_b"], row["jump_a"]
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
        plt.plot(times[self.startIndex - 1:self.takeOffIndex], torques[self.startIndex - 1:self.takeOffIndex], c='red',
                 zorder=1)
        plt.scatter(times[self.startTorqueIndex - 1], torques[self.startTorqueIndex - 1], marker='^', c='blue',
                    zorder=2)
        plt.scatter(times[self.takeOffIndex - 1], torques[self.takeOffIndex - 1], marker='^', c='blue', zorder=2)
        type = "Knee Torque vs Time"
        xlabel = 'Time(s)'
        ylabel = 'Knee Torque(N·m)'

        # 设置图形格式
        title = type + " (" + str(self.height) + "m)"  # '_' + str(self.line)
        plt.title(title, fontsize=24)
        plt.xlabel(xlabel, fontsize=16)
        fig.autofmt_xdate()  # 绘制斜的日期标签，以免它们彼此重叠
        plt.ylabel(ylabel, fontsize=16)
        plt.tick_params(axis='both', which='major', labelsize=16)
        plt.grid()

        # plt.show()
        figureName = "./chart/" + self.fileName + '_' + type + ".png"
        plt.savefig(figureName, bbox_inches='tight')

        #### fig2
        fig2 = plt.figure(dpi=128, figsize=(10, 6))
        plt.scatter(wheelLengthPs, wheelHeightPs, marker='^', c='blue', zorder=2)
        plt.plot(wheelLengths, wheelHeights, c='red', zorder=1)
        plt.plot([0.2, wheelLengths[self.highestIndex - 1]],
                 [wheelHeights[self.highestIndex - 1], wheelHeights[self.highestIndex - 1]], c='b', linestyle='--',
                 zorder=1)
        text = "heighest: " + str(('%.3f' % wheelHeights[self.highestIndex - 1])) + 'm'
        plt.text(wheelLengths[self.highestIndex - 1] + 0.2, wheelHeights[self.highestIndex - 1], text, ha='center',
                 va='bottom', fontsize=12)
        type = "Wheel Trajectory vs Distance"
        title = type + " (" + str(self.height) + "m)"  # + '_' + str(self.)
        xlabel = 'Distance(m)'
        ylabel = 'Wheel Height(m)'
        plt.xlim(0.2, max(wheelLengths) + 0.1)
        # ax = plt.gca()
        # # 移到原点
        # ax.xaxis.set_ticks_position('bottom')
        # ax.yaxis.set_ticks_position('left')
        # # ax.spines['bottom'].set_position(('data', 0))
        # ax.spines['left'].set_position(('data', 0))

        # 设置图形格式
        title = type + " (" + str(self.height) + "m)"  # '_' + str(self.line)
        plt.title(title, fontsize=24)
        plt.xlabel(xlabel, fontsize=16)
        # fig.autofmt_xdate()  # 绘制斜的日期标签，以免它们彼此重叠
        plt.ylabel(ylabel, fontsize=16)
        plt.tick_params(axis='both', which='major', labelsize=16)
        plt.grid()

        # plt.show()
        figureName = "./chart/" + self.fileName + '_' + type + ".png"
        plt.savefig(figureName, bbox_inches='tight')
