#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-04 16:55:26
#LastEditTime: 2022-01-04 17:20:57
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/Controller.py
##############

import rospy
from rospy.timer import Rate

import std_msgs.msg
# import geometry_msgs.msg
from  std_msgs.msg  import Float64MultiArray
from sensor_msgs.msg import  JointState, Imu
from geometry_msgs.msg import WrenchStamped, Twist
import tf
from URutility import *
from URScript import URScriptMove as URM
from frame_node import FrameNode as FNode


class RobotFrame(FNode):

    def __init__(self, name="basic"):
        super(RobotFrame).__init__()
        print("using %s controller now".format(name) )
        # self.init_robot(name)
        self.rate = 100
    
    #HACK: for special controller, pls revise to its own
    def init_param(self, number):
        # self._LOOP_FREQUENCY = 50
        self.sensor_number = 0
        print("finish pramas initial...")
    
    #HACK: for special controller, pls revise to its own
    def update(self):
        print("a loop work here....")

    def spin(self):
        self.init_node()
        rate = self.Rate(self.rate)
        self.init_param()
        Flag = 1
        while not self.is_shutdown() and Flag == 1:
            try:
                self.update()
                rate.sleep()
            except KeyboardInterrupt:
                sig = self.signal_shutdown("KeyboardInterrupt")
                raise
            
        self.loginfo("finished...")

    def init_node(self, name = "controller_test"):
        rospy.init_node(name)
        rospy.loginfo("init ros node: " + name)
        self.define_node_publisher()
        self.define_node_subscriber()

    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        take = self.sensor_number
        # if take > 0:
        #     joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback_joint)
        if take > 1:
        # force_sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, self.callback_ft_sensor, queue_size=1)
            # force_sub = rospy.Subscriber(
            #     "/robotiq_ft_wrench", WrenchStamped, self.callback_ft_sensor, queue_size=1)
            force_kf_sub = rospy.Subscriber(
                "robotiq_ft_kf", WrenchStamped, self.callback_ft_kf_sensor, queue_size=1)
        if take > 2:
            imu_sub = rospy.Subscriber("imu_data", Imu, self.callback_alubi, queue_size=1)
        # sensor_sub = rospy.Subscriber("/sensor_data", sensorArduino, self.callback_sensorAr, queue_size=1)

    def define_node_publisher(self):
        #publish all data to make it convenient for rosbag
        pass

        # self.vel_zd_pub = rospy.Publisher("/vzd", Float32, queue_size=1)

### call back
    def callback_joint(self, msg):
        self.read_pos_from_ur_joint(msg)

    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = dataRaw2Real(list(msg.position))
        self.now_vel = dataRaw2Real( list(msg.velocity) )
        trace_arg(self.now_ur_pos,self.now_vel)
        # self.ur.set_now_q_and_v(self.now_ur_pos,self.now_vel)

    #NOTE: frequency: 62.7hz ; topic:/robotiq_ft_wrench
    def callback_ft_sensor(self, msg):
        # self.force = [msg.Fx, msg.Fy, msg.Fz, msg.Mx, msg.My, msg.Mz]

        self.force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
    # def js_raw2real()
    def callback_ft_kf_sensor(self, msg):
        self.force_kf = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                      msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    def callback_imu(self, msg):        
        self.orient_ref = msg.orientation # in quaternion [x,y,z,w]
        self.angular_vel = msg.angular_velocity # rad/s
        self.linear_acc = msg.linear_acceleration # m/s^2

