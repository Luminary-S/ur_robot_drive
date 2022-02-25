#!/usr/bin/python
# -*- coding: utf-8 -*-
##############
# Copyright:  2021 CUHK CURI
#Author: Guangli
# Date: 2021-12-30 15:54:26
# LastEditTime: 2021-12-30 17:15:10
#LastEditors: Guangli
# Description:
# FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/move_ur3_reverse.py
##############


from os import name
from re import L
import sys
import rospy
import math
import time
import numpy as np
import quaternion
import actionlib
import tf
from URutility import *

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import std_msgs.msg
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# from URClient import *
from URClient import URClient
from URClient import ( JOINT_NAMES, JOINT_TRAJECTORY_CONTROLLERS, CARTESIAN_TRAJECTORY_CONTROLLERS, CONFLICTING_CONTROLLERS)


# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

'''
1. joint position controller 
2. joint velocity controller, for theory verification
3. cartesian position controller
4. cartesian velocity controller, for test with the force sensor
'''


class URCMD(URClient):
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        super(URCMD, self).__init__(name="urmove")
        # self.init_robot()
        self.init_param()
        print("init urcmd")

    def init_param(self):
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        pos0 = [-18.5, -12.12, 82.13, -62.42, 69.73, -91.85]
        # pos0 = [ -23.89, 0.12, 46.44, -53, 84.02, 264.36 ]
        pos_l = [18.74, -4.24, 54.28, -42.76, 106.63, -87.11]
        pos_r = [-43.17, -6.6, 64.53, -47.77, 45.3, -96.33]  # l to r : 33mm
        # pos_u = [-23.89, 0.12, 46.44, 307.83, 84.02, 264.36]
        # pos_d = [-23.89, 0.12, 46.44, 307.83, 84.02, 264.36]
        # pos_ f = [-23.89, 0.12, 46.44, 307.83, 84.02, 264.36]
        # pos_b = [-23.89, 0.12, 46.44, 307.83, 84.02, 264.36]

        self.init_pos_list = [deg2rad(i) for i in [pos0, pos_l, pos_r, pos0]]

    def go_to_init_pos(self):
        init_pose = self.init_pos_list
        print("go to init pose:", init_pose)
        # for every pose,waitin 3s to go
        dt_list = [(i+1)*3.0 for i in range(len(init_pose))]
        print("to init pose time cost:", dt_list)
        # sys.exit(1)
        return self.go_to_joint_pos(init_pose, dt_list)

    '''
    function name: go_to_pos(pos_list)
    brief: move the ur to a designed 6 joint dof position
    description: 
    param {*} pos_list:  should be in radian, [0,0,1.57,0,1,0]
    param {*} duration_list: unit is in second, [1.0,1.0,1.0]
    return {*} None
    example: self.go_to_init_pos()
    '''
    def go_to_joint_pos(self, pos_list, duration_list):
        # select joint position controller
        # trace_err()
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(
                self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        timeout = rospy.Duration(3)
        trajectory_client.wait_for_server(timeout)
        trace_err()
        rospy.sleep(5)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        for i, position in enumerate(pos_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(
            self.joint_trajectory_controller))
        # send command
        # trace_err()
        trajectory_client.send_goal(goal)
        # trace_err()
        trajectory_client.wait_for_result()
        # trace_err()
        result = trajectory_client.get_result()
        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code))
        return result
        # pass

    # 
    '''
    function name: cartesian_vel(vel)
    brief: input cartesian velocity, and publish topic to the interface
    description: used for the head adjustment
    param {*} self
    param {*} vel linear+angular in cartesian space
    return {*}
    example: vel = [0.05,0.05,0.05, 0, 0, 0]
    '''
    def cartesian_vel(self, vel):
        twist = geometry_msgs.Twist()
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.linear.z = vel[2]
        twist.angular.x = vel[3]
        twist.angular.y = vel[4]
        twist.angular.z = vel[5]

        # publish twist
        self.twist_pub.publish(twist)
        rospy.loginfo(
            " cartesion velocity setting successfully with {}".format(vel))

    def cartesian_pos(self, pos, duration = 3.0):
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[1]
        self.switch_controller(self.cartesian_trajectory_controller)
        cartesian_trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(
                self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )
        timeout = rospy.Duration(3)
        cartesian_trajectory_client.wait_for_server(timeout)
        trace_err()
        rospy.sleep(5)
        # Create and fill trajectory goal
        goal = FollowCartesianTrajectoryGoal()
        point = CartesianTrajectoryPoint()
        q = quaternion.from_euler_angles( pos[3:] )
        print(q)
        point.pose = geometry_msgs.Pose(geometry_msgs.Vector3(pos[0],pos[1],pos[3]), geometry_msgs.Quaternion(q.x,q.y,q.z,q.w) )
        point.time_from_start= rospy.Duration(duration)
        goal.trajectory.points.append(point)
        # send command
        # trace_err()
        cartesian_trajectory_client.send_goal(goal)
        # trace_err()
        cartesian_trajectory_client.wait_for_result()
        # trace_err()
        result = cartesian_trajectory_client.get_result()
        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code))
        return result

    def joint_vel(self, jspeed):
        joint_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        joint_pub.publish( Float64MultiArray(data=jspeed) )

    # NOTE: script control part
    def movej(self, l):
        cmd = "movej(" + list2str(l) + ")"
        print(cmd)
        self.publish_script_cmd(cmd)

    def servoj(self, l):
        # self.publish_script_cmd("servoj", list)
        cmd = "servoj(" + list2str(l) + ")"
        print(cmd)
        self.publish_script_cmd(cmd)

    def speedj(self, l, a=0.5,t=0.5):
        cmd = "speedj(" + list2str(l) + "," + str(a) + ")"  # + "," + str(t) +
        print(cmd)
        self.publish_script_cmd(cmd)

    def speedl(self, xd, a, t):
        cmd = "speedl(" + list2str(xd) + "," + str(a)  + "," + str(t) + ")"
        print(cmd)
        self.publish_script_cmd(cmd)

    def publish_script_cmd(self, cmd):
        self.script_publisher.publish(cmd)




if __name__ == "__main__":
    client = URCMD()

    # The controller choice is obviously not required to move the robot. It is a part of this demo
    # script in order to show all available trajectory controllers.
    trajectory_type = client.choose_controller()
    if trajectory_type == "joint_based":
        client.send_joint_trajectory()
    elif trajectory_type == "cartesian":
        client.send_cartesian_trajectory()
    else:
        raise ValueError(
            "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(
                trajectory_type
            )
        )
