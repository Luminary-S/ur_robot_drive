#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-04 19:02:38
#LastEditTime: 2022-01-04 19:02:39
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/ForceController.py
##############

import numpy as np
from URutility import *

class ForceController:

    def __init__(self):
        self.now_F = np.array( 6*[0.0] )
        self.delta_F = np.array( 6*[0.0])
        
    def get_stiffness_matrix(self):
        kfx = 1
        kfy = 1
        kfz = 1 
        ktx = 1
        kty = 1
        ktz = 1
        K = [kfx,kfy,kfz,ktx,kty,ktz]
        stiffness_matrix = np.diag(K)
        return stiffness_matrix
    
    def update(self):
        # 1. get force raw data
        F_m = np.array( [1,2,3,4,5,6] )
        # 2. estimate states

        # 3. force desired
        F_d = np.array( [1,2,4,3,4,5] )
        # 4. force bias
        delta_F = F_d - F_m
        # 5. admittance control 
        V = self.admittance_control(delta_F)
        trace_arg("target TCP vel :", V)
        return V

    def admittance_control(self, delta_F):
        V = np.array( 6 * [0.0] )
        # V1 : simple form
        S_Mat = self.get_stiffness_matrix()
        trace_arg("stiffness mat:",S_Mat)
        trace_arg("delta_F:", delta_F)
        V =  np.dot( S_Mat, delta_F )

        return V

def sim():
    fc = ForceController()
    v = fc.update() 
    # trace_arg("tcp vel cmd:")

if __name__ == '__main__':
    sim()
    


