from controller import Motor
from controller import InertialUnit
from controller import Gyro
from PID_control import *
import math




class velocity_controller:

    def __init__(self, motors, panel):
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        # 角度
        self.pitch_Kp = 10
        self.pitch_Kd = 10
        self.count = 0
        self.blance_u = 0.0
        self.blance_pid = PID_Controller(self.pitch_Kp, self.pitch_Kd)
        # 速度 参数我乱调的——hbx
        self.translation_Kp = 80
        self.translation_Kp1 = 0.0002  # 这一项确定数量级
        self.translation_Ki = 4  # 这一项决定响应时间

        self.translation_u = 0.0
        self.translation_pid = PID_Controller(self.translation_Kp, 0, self.translation_Ki)

    def setXVel(self, Ev):
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
        pitch_err = 0 - self.panel.pitch
        self.blance_pid.feedback(pitch_err)
        self.blance_u = self.blance_pid.get_u()

        translation_err = Ev / 2.0 / math.pi - self.panel.rightWheelVel  # 姑且用右轮速度做测试
        self.translation_pid.feedback(translation_err)
        self.translation_u = self.translation_pid.get_u()

        self.motors[4].setTorque(-self.blance_u + self.translation_Kp1 * self.translation_u)  # 这正负号是试出来的，我也不知道为什么——hbx
        self.motors[5].setTorque(-self.blance_u + self.translation_Kp1 * self.translation_u)

        print("b_u: %.5f" % self.blance_u)
        print("t_u: %.5f" % (self.translation_Kp1 * self.translation_u))
        print("EV: %.3f" % (Ev))
        print("V: %.3f" % (self.panel.gps_v))
        # print("Displacement: %.2f" % (self.panel.gps_dd))
        print("rWheelVel: %.5f" % (self.panel.rightWheelVel))
        # print("rWheelVelSP: %.3f" % (self.panel.samplingPeriod))

# def setAVel(self,vel):
