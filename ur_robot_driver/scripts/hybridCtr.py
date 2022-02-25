#!/home/sgl/catkin_new/venv/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-12 09:35:33
#LastEditTime: 2022-01-13 23:28:33
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/hybridCtr.py
##############


from pyexpat.errors import XML_ERROR_DUPLICATE_ATTRIBUTE
import numpy as np
import numpy.linalg as nli
from PID import PID
from sympy import *
from UtilTrans import *
from URutility import *

class HybridController(object):

    def __init__(self):
        # super().__init__()
        
        # self.pid_t = PID(1,1,1) # torque to position
        # self.pid_p = PID(1,1,1) # force to position
        self.set_ori_pid()

    def set_pid_controller(self):
        kp_t = np.diag([1,1,1])
        ki_t = np.diag([2,2,2])
        kd_t = np.diag([3,3,3])
        self.pid_t = PID( kp_t, ki_t, kd_t )
        kp_p = np.diag([1,1,1])
        ki_p = np.diag([2,2,2])
        kd_p = np.diag([3,3,3])
        self.pid_p = PID( kp_p, ki_p, kd_p )
        # return pid_p, pid

    def get_stiffness_matrix(self):
        Kxx = np.diag([1000,1000,1000])
        Kxr = np.zeros((3, 3))
        Krx = Kxr
        Krr = np.diag([10, 10, 10])
        Smat = np.vstack((np.hstack((Kxx, Kxr)), np.hstack((Kxr, Krr))))
        Cmat = nli.inv(Smat) # np.vsplit(Cmat,3)
        return Smat, Cmat

    def CTR_desired_FT(self, FT, Cmat):        
        delta_X = np.zeros((6,1))
        delta_X = np.array(FT).dot(Cmat)
        return delta_X.tolist()

    def selection_matrix(self):
        # x,y,z in constraint frame
        # constraint frame :
        #            ^ y
        #           /
        #          /
        #         o---> z
        #         |
        #         |
        #         v  x

        S_F = np.array( [0,0,1] ) # force control axis
        S_P = np.array( [1,1,0] ) # position control axis
        S_T = np.array( [1,1,0] ) # torque control axis
        S_R = np.array( [0,0,1] ) # oritation control axis
        return S_F, S_P, S_T, S_R

    def position_error_controller(self):
        X_e = []
        X_err = []

        pass

    #TODO: tf get
    def get_quat_ee_in_base(self):
        q = [1,0,0,0]# w,x,y,z
        return q

    def get_FT(self):
        F= 6*[0.0]
        return F

    def angluar_adjustment_ctr(self):
        # 1. get the quaternion of the end effector in base frame
        Q = self.get_quat_ee_in_base()
        # 2. get the force and torque from the FT sensor
        F = self.get_FT()

        # 3. target: make the touch surface normally close touching the window, if the surface is not normally close touching
        # it will: 
        # in position aspect: not reach the normal force threshold
        # in rotation aspect: since the normal force is quite big, in one side, the force is quite large and other side is quite small
        # if not touching well, two sides are not equal,  the torque in the paralle axis will be increasing big in the direction of the large force side.
        # z : FT control  vz = Kz * (Fz-Fzd)
        # y : position control dy = Ky * (y - yd), for touching yd=y0  for cleaning yd = y
        # x : position control dx = Kx * (x - xd), for touching xd=x0, for cleaning xd = x
        # rx: torque control drx = Ktx * ( Tx - Txd ) 
        # ry: torque control dry = Kty * ( Ty - Tyd )
        # rz: not controlled if no err, rz=rz0, else rz=rzd 
        # rotation in the not closely touching axis
        b_x_tool0_r = 3*[0.0]
        b_qt_tool0_r = 4*[0.0]

        return b_x_tool0_r, b_qt_tool0_r

    def environment_params(self):
        Md = np.identity(6)
        Dd = np.identity( 6)
        Kd = np.identity(6)
        Kf = np.identity(6)
        return Md, Dd, Kd, Kf

    def ContactCtr(self):
        
        # 1. info can be obtained 
        X = 6*[0.0]
        X_d = [0.1, 0., 0., 0.001, 0.001, 0.001]
        F_d = [-8.5215, 14.041, -33.782, -0.9937, 0.797, 0.5795]
        F = [2.1017, 1.0228, 1.7373, -0.0643, 0.004, -0.0433]
        e_p = X - X_d
        e_f = F - F_d
        # 2. target contact model
        x = Symbol('x')
        y = Symbol('y')
        z = Symbol('z')
        a = Symbol('a')
        b = Symbol('b')
        c = Symbol('c')
        # 1. environment model
        #minimize the moment

    def set_ori_pid(self):
        kp_t = np.diag([1.0/1e8, 1.0/1e8, 1.0/3000, 1.0/5, 1.0/6, 1.0/1e8])
        ki_t = np.diag([0, 0, 0, 0, 0, 0])
        # ki_t = np.diag([0, 0, 1.0/1000000, 1.0/1000000, 1.0/1000000, 0])
        kd_t = np.diag([0, 0, 1.0/100, 1.0/100, 1.0/100, 0])
        self.pid = PID( kp_t, ki_t, kd_t )

    def ori_pid_ctr(self, F, F_d):
        v_d = self.pid.update(F,F_d)
        delta_F = self.pid.delta_val
        abs_delta_F = np.abs(delta_F)
        diff_F =  self.pid.diff_val
        sum_delta_F = self.pid.sum_val
        v_d = get_diagonal(v_d)
        trace_round("F_0", F, "del_F", delta_F, "v_d", v_d)
        trace_round("diff_F", diff_F, "sum_delta_F", sum_delta_F)
        return v_d, abs_delta_F

    def linear_clean_ctr(self, F, F_d):
        v_d = self.pid.update(F, F_d)
        delta_F = self.pid.delta_val
        abs_delta_F = np.abs(delta_F)
        diff_F = self.pid.diff_val
        sum_delta_F = self.pid.sum_val
        v_d = get_diagonal(v_d)
        trace_round("F_0", F, "del_F", delta_F, "v_d", v_d)
        trace_round("diff_F", diff_F, "sum_delta_F", sum_delta_F)
        return v_d, abs_delta_F

    def ori_ctr(self, F, F_d, K):

        delta_F = list_minus(F, F_d)
        trace_round("F_0", F, "del_F", delta_F)
        v_d = delta_F / K
        abs_delta_F = np.abs(delta_F)
        return v_d, abs_delta_F

    def set_torque_pid(self):
        # kp_t = np.diag([1.0/1e8, 1.0/1e8, 1.0/3000, 1.0/5, 1.0/5, 1.0/1e8])
        kp_t = np.diag([1.0/3000, 1.0/5, 1.0/5])
        ki_t = np.diag([0, 0, 0])
        # ki_t = np.diag([0, 0, 1.0/1000000, 1.0/1000000, 1.0/1000000, 0])
        kd_t = np.diag([1.0/100, 1.0/100, 1.0/100])
        self.pid = PID(kp_t, ki_t, kd_t)

    def ori_torque_ctr(self, F, F_d, e_R_b, inv_e_R_b):
        head_box = 1e-3*np.array([135.0/2, 230.0/2, 130.0/2])
        P_G = 1e-3*np.array([1, 3, 78]) # in {e} frame
        mg = 1.9 * 9.85 # kg
        M_m = np.array([ 0, 0, -mg ])        
        skew_P_G = skew(P_G)
        T_G = skew_P_G.dot(inv_e_R_b.dot(M_m))  # inv_e_R_b.dot(M_m): transfer M_m into {e} frame
        skew_head_box = skew(head_box)
        f_e = np.array( F[:3] )
        T_fe = skew_head_box.dot( f_e ) 
        T_te = np.array( F[3:] )

        sum_T = T_te + T_fe + T_G

        # target is to control the sum_T_x, and sum_T_y to 0


        # 1. map the G to 
        pass

    def torque_minimize(self, F):
        pass  

def sim():
    import random
    q = []
    F = []
    imu = []
    b_x_tool0 = [-0.366, -0.053, -0.083]
    b_qt_tool0 = [0.680, 0.017, 0.733, -0.031]



if __name__ == '__main__':
    sim()
