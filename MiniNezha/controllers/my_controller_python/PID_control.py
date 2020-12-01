import numpy as np


class PID_Controller:
    """
    
    """
    err_D = 0
    err_I = 0
    Kp = 0
    Kd = 0
    Ki = 0
    u = 0
    
    def __init__(self,Kp,Kd,Ki=0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    # 仅实现了离散PID，增量式离散PID或许更好
    def feedback(self,err):
        self.err_D = err - self.err_D
        self.err_I += err
        self.u = self.Kp*err+self.Kd*self.err_D+self.Ki*self.err_I
        self.err_D = err

    def get_u(self):
        return self.u
