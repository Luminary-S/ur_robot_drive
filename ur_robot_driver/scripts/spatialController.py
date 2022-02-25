#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-05 16:07:39
#LastEditTime: 2022-01-05 16:07:39
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/spatialController.py
##############

import numpy as np
from URpacks.Universal_Robots_ROS_Driver.ur_robot_driver.scripts.URutility import quaternion2vec
import quaternion as qt
from PID import PID

class robot:
    def __init__(self, b_x_ee, b_qt_ee):
        self.b_x_ee = b_x_ee  # end-effector position in base frame
        self.b_qt_ee = b_qt_ee  # quaternion representation of orientation in base frame
    def showState(self):
        print("robot state: \n" + " tool0 position: " + self.b_x_ee, + "orientation:" + self.b_qt_ee)

# class sensor:

class Controller(object):

    def __init__(self):
        # super().__init__()
        x_b = 3*[0.0]  # vector3
        qt_b = [1.0, 0,0,0]  # quaternion
        self.rb_S_now = robot(x_b, qt_b)
        self.rb_S_past = robot(x_b, qt_b)
        self.position_pid =  PID(1,0,0)
        self.orientation_pid = PID(1,0,0)
        # pass
    
    # def window_frame(self):
    
    def set_now_robot_state(self, x, qt):
        self.rb_S_past = self.rb_S_now
        self.rb_S_now.b_x_ee = x
        self.rb_S_now.b_qt_ee =  qt
        # pass


    def orientation_ctr(self, qt, qt_d):
        #NOTE: for quaternion package, it should be [w,x,y,z], in ros, it is [x,y,z,w]
        qt = np.quaternion(qt)
        qt_d = np.quaternion(qt_d)
        delta_q = qt.inverse()*qt_d # quaternion trans
        d_q = quaternion2vec(delta_q)
        u = []
        for id in range(len(3)):
            u_tmp = self.position_pid.update(d_q[id], 0)
            u.append(u)
        return u
    
    def position_ctr(self, p, p_d ):
        u = []
        for id in range(len(3)):
            u_tmp = self.position_pid.update(p[id],p_d[id])
            u.append(u)
        return u

    def ee_ctr(self, x, q, x_d, q_d):
        u_p = self.position_ctr(x,x_d)
        u_r = self.orientation_ctr(q,q_d)
        u = np.array( u_p + u_r )

    def get_target(self):
        #TAG: in the static 
        # 1. get the joint state, get the EE in the  base frame
        # 2. from force sensor to calculate the head pose + orient
        pass
    
    # def 

    def update_old(self, x_b, qt_b, f_ft, tau_ft, angV_i, acc_i):
        # 1. set the robot state
        self.set_now_robot_state(x_b,qt_b)

        # 1. get the position, orientation of head in the window frame
        self.get_pos_ori_frame_w()


        pass

    def update(self):
        # 1. get FT wrench
        # 2. get orientation control result
        # 3. get position control result
        F_threshold = [0, 3.9, 29 ]  # noise_amplitude = [0.2, 0.3, 0.6]
        T_threshold = [ 0.165, 0.195, 0.08 ] # noise_amplitude = [0.015, 0.01, 0.04]
        
        pass

def sim():
    import random
    x_b = [-0.366, -0.053, -0.083]
    qt_b = [0.680, 0.017, 0.733, -0.031]
    

if __name__ == '__main__':
    sim()
    