'''
Author: your name
Date: 2022-01-08 02:03:16
LastEditTime: 2022-01-08 02:59:37
LastEditors: Please set LastEditors
Description: PID controller 
FilePath: /src_20211230/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/PID.py
'''
#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import warnings  
warnings.filterwarnings('error') 
import sys

class PID():

    def __init__(self, kp,ki,kd,kq=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kq = kq
        temp = np.diagonal(kp)
        self.delta_val = 0.0 * temp
        self.pre_delta_val = 0.0 * temp
        self.sum_val = 0.0 * temp
        self.diff_val = 0.0 * temp
        self.val = 0.0 * temp
        self.val_d = 0.0 * temp
        self.set_pid_params(kp,ki,kd,kq)

    def set_pid_params(self, kp,ki,kd, kq=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kq = kq
        # self.comp = comp

    def update(self, val, val_d):
        self.val = val
        self.val_d = val_d

        self.pre_delta_val = self.delta_val
        self.delta_val = self.val - self.val_d

        self.diff_val = self.delta_val - self.pre_delta_val
        # np.seterr(divide='ignore', invalid='ignore')
        # try:
        #     # with np.errstate(divide='ignore'):
        #     self.dao_diff_val = np.true_divide( self.delta_val , self.diff_val  )
        #     # if self.pre_delta_val == 0.0:
        #     #     self.dao_diff_val = 0.0
        # except RuntimeWarning as w:
        #     print("q")
        #     self.dao_diff_val = -0.001 * self.val

        self.sum_val = self.sum_val + self.delta_val

        # if self.kq != 0:
        #     print("kp:", self.kp * self.delta_val,",dao:",self.kq * self.dao_diff_val)
        # print(self.kp, self.delta_val, self.ki, self.sum_val, self.kd, self.diff_val, self.kq, self.dao_diff_val)
        u = self.kp * self.delta_val + self.ki * self.sum_val + self.kd * self.diff_val #+ self.kq * self.dao_diff_val
        return u
    
    def update_piddao(self, val, val_d):
        self.val = val
        self.val_d = val_d

        self.pre_delta_val = self.delta_val
        self.delta_val = self.val - self.val_d

        self.diff_val = self.delta_val - self.pre_delta_val
        # np.seterr(divide='ignore', invalid='ignore')
        try:
            # with np.errstate(divide='ignore'):
            self.dao_diff_val = np.true_divide( self.delta_val , self.diff_val  )
            # if self.pre_delta_val == 0.0:
            #     self.dao_diff_val = 0.0
        except RuntimeWarning as w:
            print("q")
            self.dao_diff_val = -0.001 * self.val

        self.sum_val = self.sum_val + self.delta_val
        if self.kq != 0:
            print("kp:", self.kp * self.delta_val,",dao:",self.kq * self.dao_diff_val)
        # print(self.kp, self.delta_val, self.ki, self.sum_val, self.kd, self.diff_val, self.kq, self.dao_diff_val)
        u = self.kp * self.delta_val + self.ki * self.sum_val + self.kd * self.diff_val + self.kq * self.dao_diff_val
        return u

def main():
    kp = np.diag([1,1,1])
    ki = np.diag([2,2,2])
    kd = np.diag([3,3,3])
    pid = PID(kp,ki,kd)
    val = np.diag([1,2,3])
    val_d = np.diag([1,1,1])
    
    print(pid.update(val,val_d))

if __name__ == '__main__':
    main()
