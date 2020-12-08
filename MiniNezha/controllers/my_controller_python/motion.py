from controller import Motor
from controller import InertialUnit
from controller import Gyro
import numpy as np
from PID_control import *
import math


class velocity_controller:

    def __init__(self, motors, panel):
        self.Ev = 0.0
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        # 角度
        self.pitch_Kp = 10  # 4 的时候平衡车，kp越大越稳
        self.pitch_Kd = 5
        self.count = 0
        self.blance_u = 0.0
        self.blance_pid = PID_Controller(self.pitch_Kp, self.pitch_Kd)
        # # 摆动角速度
        # self.omgz_Kp = 10
        # self.omgz_Kd = 0
        # self.omgz_u = 0.0
        # self.omgz_pid = PID_Controller(self.omgz_Kp, self.omgz_Kd, 100)

        # body速度
        self.translation_Kp = 80  #
        self.translation_Kp1 = 0.00004  # 这一项确定数量级
        self.translation_Ki = 1000      #这一项跟上一项相乘数量级需要基本不变

        self.translation_u = 0.0
        self.translation_pid = PID_Controller(self.translation_Kp, 0, self.translation_Ki)

        # # 轮子速度
        # self.wheel_Kp = 50
        # self.wheel_Kp1 = 0.0015
        # self.wheel_Ki = 10
        #
        # self.wheel_u = 0.0
        # self.wheel_pid = PID_Controller(self.wheel_Kp, 0, self.wheel_Ki)

    def calc_balance_angle(self, h):
        theta3 = np.arccos((51 * (-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                    1 / 2)) / 104)
        theta2 = np.pi - np.arccos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                           1 / 2) / 2) - theta3;
        theta1 = -np.pi / 2 + np.arccos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                                1 / 2) / 2)
        return theta1, theta2, theta3

    def setHeight(self, h):
        t1, t2, t3 = self.calc_balance_angle(h)

        self.motors[0].setPosition(t1)
        self.motors[1].setPosition(t1)
        self.motors[2].setPosition(t2)
        self.motors[3].setPosition(t2)

    def setXVel(self, Ev):  # 注意：pitch方向和车方向相反，前倾为负

        # 对比和上次的位置，用count记录累加位移的量。
        # if self.panel.gps_v < Ev:  # 之前只能做到世界坐标系下x方向的位置平衡
        #     self.count = self.count + 1
        # elif self.panel.gps_x > Ev:
        #     self.count = self.count - 1
        # # print('count:%.3f' % self.count)
        #
        # if self.count > 100:  # 抗漂移的部分
        #     self.motors[4].setTorque(0.05 * self.factor1)
        #     self.motors[5].setTorque(0.05 * self.factor1)
        #     self.factor1 = self.factor1 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
        #     self.count = self.count - 2  # 可以理解为纠正频率，减的数越小，纠正频率越高
        # elif self.count < -100:  # 抗漂移的部分
        #     self.motors[4].setTorque(-0.05 * self.factor2)
        #     self.motors[5].setTorque(-0.05 * self.factor2)
        #     self.factor2 = self.factor2 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
        #     self.count = self.count + 2  # 可以理解为纠正频率，加的数越小，纠正频率越高
        #     # print('i2%.2f' % i_2)
        # else:
        # 参数过大后降参数，减小平衡后的晃动。可以理解为抗漂移给出的力矩波形，为周期性三角锯齿波（开始瞎编）
        # if self.factor1 >= 4:
        #     factor1 = 1
        # if self.factor2 >= 4:
        #     factor2 = 1

        # PID部分
        if Ev > 0 and self.panel.bodyVel < Ev:
            self.Ev = self.panel.bodyVel + 0.1
        elif Ev < 0 and self.panel.bodyVel > Ev:
            self.Ev = self.panel.bodyVel - 0.1
        if 0 < Ev < self.Ev:
            self.Ev = Ev
        elif 0 > Ev > self.Ev:
            self.Ev = Ev
        if Ev == 0.0:
            self.Ev = 0
            angle = 0
        else:
            angle = 0#Ev/abs(Ev) * 0.05

        pitch_err = angle - self.panel.pitch
        self.blance_pid.feedback(pitch_err)
        self.blance_u = self.blance_pid.get_u()

        # omgz_err = 0 - self.panel.omega_z
        # self.omgz_pid.feedback(omgz_err)
        # self.omgz_u = self.omgz_pid.get_u()

        translation_err = self.Ev - self.panel.bodyVel  # 使用身体速度
        self.translation_pid.feedback(translation_err)
        self.translation_u = self.translation_pid.get_u()

        # wheel_err = self.Ev - self.panel.rightWheelVel*0.05  #
        # self.wheel_pid.feedback(wheel_err)
        # self.wheel_u = self.wheel_pid.get_u()

        self.motors[4].setTorque(
            -self.blance_u - 0.001 * self.omgz_u + self.translation_Kp1 * self.translation_u + self.wheel_Kp1 * self.wheel_u)
        self.motors[5].setTorque(
            -self.blance_u - 0.001 * self.omgz_u + self.translation_Kp1 * self.translation_u + self.wheel_Kp1 * self.wheel_u)

        # print("b_u: %.5f" % self.blance_u)
        # print("t_u: %.5f" % (self.translation_Kp1 * self.translation_u))
        # print("w_u: %.5f" % (self.wheel_Kp1 * self.wheel_u))
        # print("pitch_err: %.3f" % pitch_err)
        print("pitch: %.3f" % self.panel.pitch)
        print("omgz_u: %.3f" % (0.01 * self.omgz_u))
        # print("EV: %.3f" % (Ev))
        print("GPS_V: %.3f" % self.panel.gps_v)
        print("wheel_V: %.5f" % (self.panel.rightWheelVel * 0.05))
        print("body_V: %.5f" % self.panel.bodyVel)
        # print("omega_y: %.5f" % self.panel.omega_y)
        # print("omega_x: %.5f" % self.panel.omega_x)
        # print("omega_z: %.5f" % self.panel.omega_z)
        print("期望速度： %.5f" % self.Ev)
        # print("与期望速度差： %.5f" % (Ev - self.panel.rightWheelVel * 0.05))
        print("预期倾角：%.3f" % angle)
        # print("Displacement: %.2f" % (self.panel.gps_dd))
        # print("rWheelVel: %.5f" % (self.panel.rightWheelVel))
        # print("rWheelVelSP: %.3f" % (self.panel.samplingPeriod))

# def setAVel(self,vel):
