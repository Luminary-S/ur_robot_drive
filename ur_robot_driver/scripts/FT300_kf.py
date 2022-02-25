#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-03 20:20:41
#LastEditTime: 2022-01-03 20:20:42
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/path_publish.py
##############

from tkinter import N
from turtle import update
from geometry_msgs.msg import WrenchStamped
from os import name
import rospy, time
from rospy.timer import Rate
from URutility import publish_force, trace_arg, trace_round
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from filter import Filter
import numpy as np
import threading

class FTKFListener(object):
    def __init__(self):
        self.f = []  
        self.FT_kf_listener()

    def FT_kf_listener(self):
        ft_kf_sub = rospy.Subscriber(
            "/robotiq_ft_kf", WrenchStamped, self.callback_ft_kf_sensor, queue_size=1)

    def callback_ft_kf_sensor(self,msg):
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        self.f = force
        return force

class FTKF(object):
    
    def __init__(self):
        self.f = Filter()
        self.thread = threading.Thread(
            target=self.spin, daemon=True)  # daemon=True, MainThread 结束，子线程也立马结束

    def run(self):
        alive = self.thread.start()
        return alive

    def ros_init(self):
        self.node_subscriber()
        self.node_publisher()
    
    def init_node(self):
        self.name = "ft300_kf"
        rospy.init_node("ft300_kf")

    def node_subscriber(self):
        force_sub = rospy.Subscriber(
            "/robotiq_ft_wrench", WrenchStamped, self.callback_ft_sensor, queue_size=1)
    def node_publisher(self):
        self.ft_pub = rospy.Publisher("robotiq_ft_kf", WrenchStamped, queue_size=1)

    #NOTE: frequency: 62.7hz ; topic:/robotiq_ft_wrench
    def callback_ft_sensor(self, msg):
        # self.force = [msg.Fx, msg.Fy, msg.Fz, msg.Mx, msg.My, msg.Mz]
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        self.f.add(force)

    def publish(self, F, name="robotiq_ft_KF"):
        pub = self.ft_pub
        publish_force(pub,F,name)

    def init_param(self):
        # self.f = Filter()
        SAMPLE_LEN = 10 
        l = self.f.buf[-SAMPLE_LEN:] if len(self.f.buf)>SAMPLE_LEN else self.f.buf
        # print(len(l))
        # print(np.average(l,axis=0))
        F0_avg = np.average(l, axis=0).tolist()
        cov0 = np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
        initstate = F0_avg
        Transition_Matrix = 1.0*np.eye(6)
        Observation_Matrix = 1.0*np.eye(6)
        initcovariance = np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
        transistionCov = np.diag(
            [0.008, 0.008, 0.008, 0.0021, 0.002, 0.002])
        observationCov = np.diag(
            [1.00, 1, 1, 0.25, 0.25, 0.25])
        self.f.kf_define(Transition_Matrix,Observation_Matrix,initstate,initcovariance,transistionCov,observationCov)

        return F0_avg,cov0

    # NOTE: refer to [[Exp_kalmanfilter]]
    def update(self, F,cov, M): 
        # trace_round("F_m", M, "F_i", F)
        # print(cov)
        F2, cov2 = self.f.kf_update(F, cov, M)
        # print(F,cov)
        # smoothed = self.f.kf.em(F).smooth(F)
        # print(smoothed)
        # filtered = kf.em(values).filter(values)[0]
        # self.publish(F2)
        return F2,cov2

    def spin(self):
        rate = rospy.Rate(60)
        # trace_arg("111")
        self.ros_init()
        rospy.sleep(1)
        F0, cov0 = self.init_param()
        cov = cov0
        F = F0
        while not rospy.is_shutdown():
            # print("==start loop===")
            try:
                F_m = self.f.buf[-1]
                cov_i = cov
                F_i = F
                # trace_round("F_m",F_m,"F_i",F_i)
                F, cov = self.update(F_i, cov_i, F_m)
            except rospy.exceptions.ROSException as err:
                print(err)
            rate.sleep()

def main():
    ft = FTKF()
    ft.init_node()
    ft.ros_init()
    rate = rospy.Rate(60)
    rospy.sleep(1)
    F0, cov0 = ft.init_param()
    cov = cov0
    F = F0
    while not rospy.is_shutdown():
        # print("==start loop===")
        try:
            F_m = ft.f.buf[-1]            
            cov_i = cov
            F_i = F 
            # print(F_m)
            # print(cov_i)
            F,cov = ft.update(F_i,cov_i, F_m)
            # print(F)
            # print(cov)
            ft.publish(F)
        except rospy.exceptions.ROSException as err:
            print(err)
        rate.sleep()

if __name__ == '__main__':
    main()
    