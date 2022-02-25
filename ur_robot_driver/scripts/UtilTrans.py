#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-14 18:20:04
#LastEditTime: 2022-01-14 18:22:33
#LastEditors: Guangli
#Description: math utility for the matrix and coodinate transformation
#FilePath: /ur_robot_driver/scripts/UtilTrans.py
##############

import math
# import math.pi as M_PI
import numpy as np
import quaternion
from pyquaternion import Quaternion

M_PI = math.pi

############### math trans #######################
def deg2rad(p_list):
    return [i/180.0*math.pi for i in p_list]

def dataRaw2Real(l):
    return [l[2], l[1], l[0], l[3], l[4], l[5]]

def list_minus(a,b):
    return  np.array(a) - np.array(b)


def list_add(a, b):
    return np.array(a) - np.array(b)

def V_d_limitation( V, limit_V ):
    
    # for i in limit_V
    V_r = []
    sign_V = np.sign(V)
    print(sign_V)
    delta_V = np.abs(np.array(V)) - limit_V
    print(delta_V)
    for i in range(len(delta_V)):
        if delta_V[i] >= 0:
            V_r.append(limit_V[i]*sign_V[i])
        else:
            V_r.append(V[i])
    return V_r

def minimum_degree_distance(d1, d2):
    delta_d = d1 - d2
    return 

def to_degrees(radians):
    return radians * 180.0 / M_PI

def normalize_angle_positive(angle):
    return math.fmod(math.fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI)

def normalize_angle(angle):
    a = normalize_angle_positive(angle)
    if (a > M_PI):
      a -= 2.0 *M_PI
    return a

def shortest_angular_distance(from_a, to_a):
    return normalize_angle(to_a-from_a)


def get_diagonal(mat):
    return np.diagonal(mat)

#Refer: Axis-Angle https://zhuanlan.zhihu.com/p/45404840, https://blog.csdn.net/u014170067/article/details/83834043, https://blog.csdn.net/weixin_43455581/article/details/108413001
#quaternion in tf: x,y,z,w; in numpy-quaternion: w,x,y,z, here in the numpy-quaternion form
def quaternion2axisangle(q):
    theta = math.acos(-q[0]) * 2.0
    sin_theta = math.sin(theta/2.0)
    return [-q[1]/sin_theta * theta, -q[2]/sin_theta*theta, -q[3]/sin_theta*theta]

def quaternion2rot(q):
    quat = Quaternion(q)
    rot = quat.rotation_matrix
    return rot

def quaternion2invRot(q):
    quat = Quaternion(q).inverse
    rot = quat.rotation_matrix
    return rot

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])
#NOTE: x,y,z,w:tf form ; w,x,y,z: pyquaternion,Quaternion and numpy-quaternion; scipy: x,y,z,w form
def trans_between_tfQu_pyQu(qu):
    return [qu[3],qu[0],qu[1],qu[2]]

def quaternion2vec(q):
    return [q.x, q.y, q.z]

def force_measure_to_in_force_frame(F):
    return [-F[0],-F[1],-F[2],-F[3],-F[4],-F[5]]

'''
function name: angular_velocity_2_quaternionDiff(W)
brief: get the differential  quaternion with the angular velocity of W
description: 
param W: desired angluar velocity, [1,2,3] x,y,z
param qu: quaternion of temporary rotation [-0.130, -0.653, -0.046, 0.745] x,y,z,w:tf form
return {*} desired quaternion velocity
example: 
'''
def angular_velocity_2_quaternionDiff(W, qu):
    q0 = qu[3]
    qr = qu[:-1] 
    diff_qu_d = np.zeros((4,1))
    trans_mat = np.matrix((4,3))
    diff_qu_d = trans_mat.dot(W)
    return diff_qu_d


def J_in_static_frame(rot):
    J1 = np.hstack((rot, np.zeros((3, 3))))
    J2 = np.hstack((np.zeros((3, 3)), rot))
    J = np.vstack((J1,J2))
    return J