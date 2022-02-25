#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-03 16:48:33
#LastEditTime: 2022-01-03 16:48:34
#LastEditors: Guangli
#Description: Urscript control, tested in ur3 with Cab3.1 software3.9, ubuntu18.04, ROS 1 melodic
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/URScript.py
##############

# from math import degrees
import rospy
import std_msgs.msg
from  std_msgs.msg  import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import tf
# import tf2_py
from URutility import *
from UtilTrans import *
import numpy as np
# from pyquaternion import Quaternion

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class URScriptMove(object):

    def __init__(self):
        # super(URScriptMove).__init__()
        self.name = "urscript_move"
        # self.frequency = 50
        # self.init_robot()
    
    def init_node(self, name = "urscript_test"):
        rospy.init_node(name)
        rospy.loginfo("init ros node: " + name)
        # self.set_rate(self.frequency)
        # self.rate = rospy.Rate(self.frequency)
        # self.define_node_publisher()
        # self.define_node_subscriber()

    def set_rate(self, frequency=50):
        self.frequency = frequency

    def init_robot(self):
        """Make sure the robot is booted and ready to receive commands"""
        # self.basic_client()

        # wait for tf created
        self.script_publisher = rospy.Publisher(
            "/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)
        self.twist_pub = rospy.Publisher(
            "/twist_controller/command", Twist, queue_size=1)
        self.joint_pub = rospy.Publisher(
            "/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(
            'world', "tool0", rospy.Time(0), rospy.Duration(3.0))
        self.ur_js_states_sub = rospy.Subscriber(
            "/joint_states", JointState, self.callback_js_state)

    def get_tf_pose_rot(self, baseFrame="base", targetFrame="tool0_controller"):
        position, quaternion = self.tf_listener.lookupTransform(
            baseFrame, targetFrame, rospy.Time(0))
        quat = trans_between_tfQu_pyQu(quaternion)
        # print(quat)
        return position,quat

    def callback_js_state(self, msg):
        self.read_pos_from_ur_joint(msg)    

    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = urtopic_list2right(list(msg.position))
        self.now_vel = urtopic_list2right(list(msg.velocity))

    #NOTE: Move to position (linear in joint-space)
    def movej(self, l, a=1.4, v=1.05, t=0, r=0):
        param = "a="+ str(a) + ",v=" + str(v) + ",t=" + str(t)+ ",r=" + str(r)
        cmd = "movej(" + list2str(l) + "," + param + ")"
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)
        return cmd

    #NOTE: Move to position (linear in tool-space), based on base frame
    def movel(self, l, a=1.4, v=1.05, t=0, r=0):
        # If it were specified the command would ignore the a and v values
        param = "a=" + str(a) + ",v=" + str(v) + ",t=" +   str(t) + ",r=" + str(r)
        cmd = "movel(p" + list2str(l) + "," + param + ")"
        # cmd = "movel(p%s, a=%s, v=%s, t=%s, r=%s)" % (
        #     l, a, v, t, r)
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)
    '''
    function name: relative_movel( l)
    brief: get the tcp (p,r) w.r.t base from tf, then add a translation to the p, then put the new (p,r) to the movel cmd 
    description: linear movement in the tcp frame
    param {*} self
    param {*} l: tanslation, here only in 3 len list
    return {*}
    example: 
    '''
    # @display_time
    def relative_movel(self, l):
        # l = [0,0,0.1]
        position, quaternion = self.get_tf_pose_rot()  # find the closest point, need the tf from base to tcp
        # trace_arg("transition: ", position, " rotation: ", quaternion)
        trans_d = [ position[i] + l[i] for i in range(3) ] # get a new trans 
        rot_d = quaternion2axisangle(quaternion)# in equal axis-angle form [rx,ry,rz] 
        # trace_arg("transition: ",trans_d," rotation: ",rot_d)
        self.movel(trans_d+rot_d, 0.4, 0.1)

    def servoj(self, l):
        # self.publish_script_cmd("servoj", list)
        cmd = "servoj(" + list2str(l) + ")"
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)

    #NOTE: stop when the robot reaches the speed 
    # @display_time
    def speedj(self, l, a=1.4, t=20):
        param = "a=" + str(a) +  ",t=" + str(t) 
        cmd = "speedj(" + list2str(l) + "," + param + ")"  # + "," + str(t) +
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)

    #NOTE: relate to base frame, not base_link or others
    # t means how long the velocity keeps.
    # @display_time
    def speedl(self, xd, a, t=5, aRot=0.1):

        # unit: [m/s,rad/s], m/s^2, rad/s^2
        cmd = "speedl(" + list2str(xd) + "," + str(a) + "," + str(t) + ")"
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)

    def speedl_tool(self, dx, a, t=5, aRot=0.1): 
        from scipy.spatial.transform import Rotation as R
        # unit: [m/s,rad/s], m/s^2, rad/s^2
        position, quaternion = self.get_tf_pose_rot()
        trace_round("now pose", position,"now axis angle", quaternion2axisangle(quaternion))
        rot1 = quaternion2rot(quaternion) # np matrix, 3*3
        J1 = J_in_static_frame(rot1)
        dx_base1 = J1.dot(np.array(dx))  # 3*6 x 6*1
        trace_round("dx_base1", dx_base1)

        cmd = "speedl(" + list2str(dx_base1) + "," + str(a) + "," + str(t) + ")"

        # trace_cmd(cmd)
        self.publish_script_cmd(cmd)
        return dx_base1

    def stopj(self,a=2):
        # a unit: rad/s^2
        cmd = "stopj("+ str(a) + ")"
        trace_cmd(cmd)
        self.publish_script_cmd(cmd)

    def publish_script_cmd(self, cmd):
        self.script_publisher.publish(cmd)

    def joint_vel(self, jspeed):
        # unit: radian
        trace_arg("joint vel cmd: ",jspeed)
        self.joint_pub.publish( Float64MultiArray(data=jspeed) )

    # def cartesian_twist(self, vel):
    #     twist = Twist()
    #     twist.linear.x = vel[0]
    #     twist.linear.y = vel[1]
    #     twist.linear.z = vel[2]
    #     twist.angular.x = vel[3]
    #     twist.angular.y = vel[4]
    #     twist.angular.z = vel[5]

    #     # publish twist
    #     self.twist_pub.publish(twist)
    #     rospy.loginfo(
    #         " cartesion velocity setting successfully with {}".format(vel))

    def go_to_init(self):
        # down installation
        pos0 = [-18.5, -12.12, 82.13, -62.42, 69.73, -91.85]
        # pos0 = [ -23.89, 0.12, 46.44, -53, 84.02, 264.36 ]
        pos_l = [18.74, -4.24, 54.28, -42.76, 106.63, -87.11]
        pos_r = [-43.17, -6.6, 64.53, -47.77, 45.3, -96.33]  # l to r : 33mm
        self.movej(deg2rad(pos0))

    def go_to_init_upinstall(self):
        pos0 = [-1.89, -54.04, 47, -172.84, 257.04, -83.96]
        return  self.movej(deg2rad(pos0))
        