from controller import Motor
from controller import InertialUnit
from controller import Gyro
from PID_control import *
import math


class Point(object):
    def __init__(self, xParam=0.0, yParam=0.0, zParam=0.0):
        self.x = xParam
        self.y = yParam
        self.z = zParam

    # def __str__(self):
    #     return "(%.2f, %.2f, %.2f)" % (self.x, self.y,self.z)

    def diff(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return Point(dx, dy, dz)

    def distance(self, other):
        return math.sqrt(self.diff(other).x ** 2 + self.diff(other).y ** 2 + self.diff(other).z ** 2)

    def sum(self, pt):
        newPt = Point()
        xNew = self.x + pt.x
        yNew = self.y + pt.y
        zNew = self.z + pt.z
        return Point(xNew, yNew, zNew)

    def multiple(self, pt):
        newPt = Point()
        xNew = self.x * pt.x
        yNew = self.y * pt.y
        zNew = self.z * pt.z
        return Point(xNew, yNew, zNew)

    def dotMultiple(self, pt):
        newPt = Point()
        xNew = self.x * pt.x
        yNew = self.y * pt.y
        zNew = self.z * pt.z
        return xNew + yNew + zNew


class panel:
    # 举例，若调用gps，则,panel.updateGPS(),panel.gps_x
    def __init__(self, gps, gyro, imu, TIME_STEP):
        self.x = Point(1, 0, 0)  # 车身参考系在世界坐标系下的初始位置
        self.y = Point(0, 1, 0)
        self.z = Point(0, 0, 1)
        self.dir = 1  # 默认往前走

        self.TIME_STEP = TIME_STEP

        self.gps = gps
        self.gps_dx, self.gps_dy, self.gps_dz, self.gps_dd = 0, 0, 0, 0  # 与前一个timestep的位置差,通过gps获取的数据
        self.gps_x, self.gps_y, self.gps_z, self.gps_d = 0, 0, 0, 0  # 从GPS获取的位置
        self.x_last, self.y_last, self.z_last, self.d_last = 0, 0, 0, 0  # 前一个timestep的位置
        self.gps_dir = Point(self.gps_x, self.gps_y, self.gps_z)
        self.gps_ddir = Point(self.gps_dx, self.gps_dy, self.gps_dz)
        self.gps_dir_last = Point(self.x_last, self.y_last, self.z_last)

        self.gyro = gyro
        self.omega_x, self.omega_y, self.omega_z = 0, 0, 0

        self.imu = imu
        self.roll, self.pitch, self.yaw = 0, 0, 0

    def upadteDirection(self):  # TODO 这里不太确定，先假设水平了——hbx
        self.x = Point.multiple(self.x, Point(math.cos(self.yaw), math.sin(self.yaw), 0))
        self.y = Point.multiple(self.y, Point(-math.sin(self.yaw), math.cos(self.yaw), 0))
        self.z = (0, 0, 1)
        # self.car_dir = (self.x,self.y,self.z)

    def updateGPS(self):  # gps_y is the height of robot
        self.x_last, self.y_last, self.z_last = self.gps_x, self.gps_y, self.gps_z
        self.gps_x, self.gps_y, self.gps_z = self.gps.getValues()

        self.gps_dir = Point(self.gps_x, self.gps_y, self.gps_z)
        self.gps_dir_last = Point(self.x_last, self.y_last, self.z_last)

        self.gps_ddir = Point.diff(self.gps_dir, self.gps_dir_last)

        # self.gps_dx = self.gps_x - self.x_last
        # self.gps_dy = self.gps_y - self.y_last
        # self.gps_dz = self.gps_z - self.z_last

        # 位置差 TODO 旋转180°的时候dir会反复横跳，要修改
        if Point.dotMultiple(self.gps_ddir, self.x) !=0:
            self.dir = Point.dotMultiple(self.gps_ddir, self.x) / abs(
                Point.dotMultiple(self.gps_ddir, self.x))  # 计算位移向量和方向向量夹角
        self.gps_dd = self.dir * Point.distance(self.gps_dir, self.gps_dir_last)

        # 速度 用gps算速度主要是想着不会有打滑的问题
        self.gps_v = self.gps_dd / self.TIME_STEP

    def updateIMU(self):  # pitch is the angle in rad w.r.t z-axis
        self.roll, self.pitch, self.yaw = self.imu.getRollPitchYaw()

    def updateGyro(self):
        self.omega_x, self.omega_y, self.omega_z = self.gyro.getValues()


class balance_controller:

    def __init__(self, motors, panel):
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        self.Kp = 10
        self.Kd = 10
        self.blance_pid = PID_Controller(self.Kp, self.Kd)

        self.count = 0
        self.blance_u = 0

    def balance(self):

        # 对比和上次的位置，用count记录累加位移的量。
        if self.panel.gps_dd < 0:  # 之前只能做到世界坐标系下x方向的位置平衡
            self.count = self.count + 1
        elif self.panel.gps_dd > 0:
            self.count = self.count - 1
        # print('count:%.3f' % self.count)

        if self.count > 100:  # 抗漂移的部分
            self.motors[4].setTorque(0.05 * self.factor1)
            self.motors[5].setTorque(0.05 * self.factor1)
            self.factor1 = self.factor1 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
            self.count = self.count - 2  # 可以理解为纠正频率，减的数越小，纠正频率越高
        elif self.count < -100:  # 抗漂移的部分
            self.motors[4].setTorque(-0.05 * self.factor2)
            self.motors[5].setTorque(-0.05 * self.factor2)
            self.factor2 = self.factor2 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
            self.count = self.count + 2  # 可以理解为纠正频率，加的数越小，纠正频率越高
            # print('i2%.2f' % i_2)
        else:
            # 参数过大后降参数，减小平衡后的晃动。可以理解为抗漂移给出的力矩波形，为周期性三角锯齿波（开始瞎编）
            if self.factor1 >= 4:
                factor1 = 1
            if self.factor2 >= 4:
                factor2 = 1

            # PID部分
            err = 0 - self.panel.pitch
            self.blance_pid.feedback(err)
            self.blance_u = self.blance_pid.get_u()

            self.motors[4].setTorque(-self.blance_u)
            self.motors[5].setTorque(-self.blance_u)

        # print("Y: %.3f" % (self.panel.gps_y))
        print("X: %.3f" % (self.panel.gps_x))
        # print("Z: %.3f" % (self.panel.gps_z))
        # print("dD: %.3f" % (self.panel.gps_dd))
        # print("pitch: %.3f torque: %.3f" % (self.panel.pitch, self.blance_u))
        # print("Yaw: %.3f" % (self.panel.yaw))
        print("Dir: %.3f" % (self.panel.dir))

# class velocity_controller
# def setXVel(self,omega):

# def setAVel(self,vel):
