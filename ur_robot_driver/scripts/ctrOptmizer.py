#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-25 02:30:49
#LastEditTime: 2022-01-25 02:37:51
#LastEditors: Guangli
#Description: 
#FilePath: /URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/ctrOptmizer.py
##############
import numpy as np
from cvxopt import matrix, solvers


def optimizer():
    
    pass

A = matrix([[-1.0, -1.0, 0.0, 1.0], [1.0, -1.0, -1.0, -2.0]])
b = matrix([1.0, -2.0, 0.0, 4.0])
c = matrix([2.0, 1.0])

sol = solvers.lp(c, A, b)

print(sol['x'])
print(np.dot(sol['x'].T, c))
print(sol['primal objective'])
