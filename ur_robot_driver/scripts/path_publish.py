#!/usr/bin/python
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

from geometry_msgs.msg import WrenchStamped
from os import name
import rospy, time
from rospy.timer import Rate
# import tf2_ros
# from tf2_geometry_msgs import PointStamped
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import  JointState
from nav_msgs.msg import Path
import tf
from URutility import *

class PathRviz(object):

    def __init__(self):
        self.name = "TCPodom"
        rospy.init_node("rviz_path")
        # self.buf = tf2_ros.Buffer()
        self.tf_listener = tf.TransformListener()
        # self.subscriber()
        self.publisher()

    def subscriber(self):
        joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self.callback_joint)
    def publisher(self):
        self.path_pub = rospy.Publisher("tool0_traj", Path, queue_size=10 )

    def callback_joint(self, msg):
        self.read_pos_from_ur_joint(msg)

    def read_pos_from_ur_joint(self, msg):
        self.now_ur_pos = dataRaw2Real(list(msg.position))
        self.now_vel = dataRaw2Real(list(msg.velocity))

    def pack_nav(self, msg, trans, quat, name):
        # msg = Path()
        # pose = PointStamped()
        pose = PoseStamped()
        current_time = rospy.Time.now()
        pose.header.stamp = current_time
        pose.header.frame_id = name
        x,y,z = trans
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = -quat[0]
        pose.pose.orientation.y = -quat[1]
        pose.pose.orientation.z = -quat[2]
        pose.pose.orientation.w = -quat[3]

        msg.header.stamp = current_time
        msg.header.frame_id = name
        msg.poses.append(pose)
        return msg

    def update_tool0_controller(self, path, name="tool0_controller"):
        # subscribe tf relationship of base to tool0_controller/ robotiq_ft_frame_id
        
        # t = self.buf.lookup_transform(# 'world', 'tool0_controller', rospy.Time(0))
        # self.tf_listener.waitForTransform('world', name, rospy.Time(0), rospy.Duration(3.0))
        # t = self.tf_listener.getLatestCommonTime('world', 'tool0_controller')

        position, quaternion = self.tf_listener.lookupTransform('world', name, rospy.Time(0))
        # print(position,quaternion)
        # (trans, rot) = self.tf_listener.lookupTransform(
        #     'world', 'tool0_controller', rospy.Time(0))
        # pack  trans and rot data into path nav_msg
        # path = Path()
        # print(t)
        # position = t.transform.translation
        # quaternion = t.transform.rotation
        path = self.pack_nav(path, position, quaternion, "world") # pack the pose relate frame
        # publish the msg
        self.path_pub.publish(path)
        # return too0_controller_path

def main():
    pr = PathRviz()
    
    rate = rospy.Rate(10)
    too0_controller_path = Path()
    rospy.sleep(5)
    while not rospy.is_shutdown():
        try:
            pr.update_tool0_controller(too0_controller_path, "tool0_controller")
        except rospy.exceptions.ROSException as err:
            print(err)
        rate.sleep()

if __name__ == '__main__':
    main()
    