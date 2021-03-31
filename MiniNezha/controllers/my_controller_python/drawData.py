import csv
from matplotlib import pyplot as plt

class drawer:
    def __init__(self):
        self.bodyHeight = []
        self.opt_vel = 0
        self.obj_val = 0
        self.a, self.b, self.c, self.d = 0, 0, 0, 0
        self.height = 0
        self.line = 0

        self.isPointPos=False
        self.fileName = ''

    def changeArgs(self, height, line):
        self.height = height
        self.line = line
        if height == 0.2:
            csvName = 'args_11.341-h=0.2.csv'
        elif height == 0.3:
            csvName = 'args_16.684-h=0.3.csv'
        elif height == 0.4:
            csvName = 'args_24.735-h=0.4.csv'
        else:
            csvName = ''
        with open(csvName, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for i, rows in enumerate(reader):
                if i == line:
                    row = rows
        self.opt_vel, self.obj_val, self.d, self.c, self.b, self.a=row[0],row[1],row[2],row[3],row[4],row[5]

        file_handle = open('args.txt', mode='w')
        file_handle.writelines(['{\'opt_vel\'', ': ', str(self.opt_vel),', ',
                                '\'objective_value\': ',str(self.obj_val),', ',
                                '\'jump_d\': ', str(self.d),', ',
                                '\'jump_c\': ', str(self.c), ', ',
                                '\'jump_b\': ', str(self.b), ', ',
                                '\'jump_a\': ', str(self.a), '}'])
        file_handle.close()


    def drawData(self):
        with open(self.fileName) as f:
            reader = csv.reader(f)
            header_row = next(reader)

            times,wheelHeights = [], []
            for row in reader:

                time = int(row[0])
                times.append(time)

                wheelHeight = int(row[1])
                wheelHeights.append(wheelHeight)

        # 根据数据绘制图形
        fig = plt.figure(dpi=128, figsize=(10, 6))
        plt.plot(times, wheelHeights, c='red')
        #plt.plot(dates, lows, c='blue')

        # 设置图形格式
        plt.title("Daily high and low temperatures - 2014", fontsize=24)
        plt.xlabel('', fontsize=16)
        fig.autofmt_xdate()  # 绘制斜的日期标签，以免它们彼此重叠
        plt.ylabel("Temperture (F)", fontsize=16)
        plt.tick_params(axis='both', which='major', labelsize=16)

        plt.show()
        plt.savefig("tmp3.png", bbox_inches='tight')

