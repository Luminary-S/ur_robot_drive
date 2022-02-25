#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2021-12-31 17:24:40
#LastEditTime: 2021-12-31 17:24:41
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/URutility.py
##############

import rospy
from geometry_msgs.msg import WrenchStamped,TwistStamped, PoseStamped

import time
# from tokenize import String
# from warnings import resetwarnings
# import numpy as np
# import quaternion


############### trace #######################
def display_time(func):
    def wrapper(*args):
        t1 = time.time()
        # print('t1 = %.2f' % func(*args))
        res = func(*args)
        t2 = time.time()
        print('time cost: %.6f' % (t2-t1))
        return res
    return wrapper


def traceFunc(CLOSE):
    def func_outer(trace_function):
        def func_inner(*args, **kwargs):
            if CLOSE == 1:  # means not print
                return
            else:
                trace_function(*args, **kwargs)
        return func_inner
    return func_outer


@traceFunc(0)
def trace_arg(*args):
    # print(args[1], len(args))
    print(" ".join(map(str, args)))

@traceFunc(0)
def trace_err(*args):
    print("ERR!!!:")
    trace_arg(*args)
    print("-----------", time.time())

@traceFunc(0)
def trace_cmd(cmd):
    trace_arg(cmd)

@traceFunc(0)
def trace_loop(i):
    print("======= loop: %d ======" %i)

import numpy as np
@traceFunc(0)
def trace_round(*args):
    res = []
    res.append("---\n")
    for item in args:
        if isinstance(item, list) or isinstance(item, np.ndarray):
            l = [float("%.4f" % float(i)) for i in item]
            res.append(l)
            res.append("\n")
        elif isinstance(item, str):
            res.append(item)
            res.append(": ")
        elif isinstance(item, (int, float, bool)):
            res.append(str(item))
            # res.append(": ")    
    # res.append("---")
    res = res[:-1]
    trace_arg(*res)


#TAG:############## ROS #######################
class Pub(object):

    def __init__(self, name, type):
        # super().__init__()
        self.name = name
        self.type = type
        self.pub = self.define_pub(self.type, self.name)

    def define_pub(self, type, name):
        if type == "force":
            return define_force_pub(name)
        elif type == "twist":
            return define_twist_pub(name)
        
    def publish_data(self, data):
        p = self.pub
        name = self.name
        type = self.type
        if type == "force":
            publish_force(p, data, name)
        elif type == "twist":
            publish_twist(p, data, name)


def exp_publish(*arg):
    ln = len(arg)//2
    if len(arg) % 2 == 0 and ln > 0:
        for i in range(ln):
            # print(arg[i*2])
            # print(arg[i*2+1])
            arg[i*2].publish_data(arg[i*2+1])

def define_force_pub(name):
    return rospy.Publisher(name, WrenchStamped, queue_size=1)

def publish_force(pub, F, name):
    w = WrenchStamped()
    current_time = rospy.Time.now()
    w.header.stamp = current_time
    w.header.frame_id = name
    # x,y,z = trans
    # print(F[0])
    w.wrench.force.x = F[0]
    w.wrench.force.y = F[1]
    w.wrench.force.z = F[2]
    w.wrench.torque.x = F[3]
    w.wrench.torque.y = F[4]
    w.wrench.torque.z = F[5]
    pub.publish(w)

def define_twist_pub(name):
    return rospy.Publisher(name, TwistStamped, queue_size=1)

def publish_twist(pub, TW, name):
    w = TwistStamped()
    current_time = rospy.Time.now()
    w.header.stamp = current_time
    w.header.frame_id = name
    # x,y,z = trans
    w.twist.linear.x = TW[0]
    w.twist.linear.y = TW[1]
    w.twist.linear.z = TW[2]
    w.twist.angular.x = TW[3]
    w.twist.angular.y = TW[4]
    w.twist.angular.z = TW[5]
    pub.publish(w)

def publish_pose(pub, trans, quat, name):
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

    pub.publish(pose)


def sleep(t):
    rospy.sleep(t)

def urtopic_list2right(l):
    return [l[2], l[1], l[0], l[3], l[4], l[5]]

#TAG:############# UR ##################
def cmd_script(cmd_str, l):
    # cmd = "movej", l = [1,2,3,4,5,6]
    out = cmd_str + "(" + list2str(l) + ")"
    return out

def list2str(l):
    s = ','.join(map(str, l))
    return "["+s+"]"


# def main():
#     q1 = np.quaternion(1, 2, 3, 4)
#     q1.normalized()
#     r1 = quaternion.as_rotation_matrix(q1) 
#     print(r1)

# if __name__ == '__main__':
#     main()
    
