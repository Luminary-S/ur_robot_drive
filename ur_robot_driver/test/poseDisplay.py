#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-02-21 10:29:46
#LastEditTime: 2022-02-21 10:30:20
#LastEditors: Guangli
#Description: show the vector in the rviz
#FilePath: /URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/poseDisplay.py
##############


import rospy
from geometry_msgs.msg import PoseStamped
from URutility import *

class PoseDisplay(object):
    
    def __init__(self, name):
        # super().__init__()
        
        # self.pid_t = PID(1,1,1) # torque to position
        # self.pid_p = PID(1,1,1) # force to position
        # pass
        self.name = name
        self.define_pub(name)

    def define_pub(self, name):
        name = "/rviz_display/" + name
        self.posePub = rospy.Publisher(
            name, PoseStamped, queue_size=1)

    def decode_vector_to_pose(self, vec):
        trans = []
        quat = []
        return trans, quat

    def publish(self, vec):
        # 1. get the value into a pose message
        trans, quat = self.pack_into_msg(vec)
        # 2. publish the msg as ros topic
        # posePub = rospy.Publisher(
        #     "/rviz_display/", Float64MultiArray, queue_size=1)
        # self.posePub.publish(msg)
        publish_pose(self.posePub, trans, quat, name)

    # def define_source(self, value)
