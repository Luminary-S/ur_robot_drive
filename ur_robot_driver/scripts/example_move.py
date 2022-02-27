#!/home/sgl/catkin_new/venv/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright: © 2021 CUHK CURI
#Author: Guangli
#Date: 2021-12-30 16:44:34
#LastEditTime: 2021-12-30 17:11:10
#LastEditors: Guangli
#Description: example to using the move_ur3_reverse.py
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/example_move.py
##############

# from move_ur3_reverse import URCMD
# from move_ur3_reverse_copy import URCMD
import time,rospy
from URScript import URScriptMove as URM
import sys
# from test.test_kalman import KalmanFilter

def test_joint_pose_move():
    tc = URM()
    tc.init_node("ur3_script_move")    
    tc.init_robot()
    rospy.sleep(0.5)
    # res1 = tc.go_to_init_pos()
    res1 = tc.go_to_init_upinstall()
    # print(res1)
    # tc.send_program_srv.call()
    # rospy.sleep(0.5)
    # tc.switch_controller("twist_controller")
    # rospy.sleep(2)
    # res2 = tc.cartesian_vel([0.1,0,0,0,0,0])
    # print(res2)
    # rospy.sleep(2)
    # res3 = tc.cartesian_vel([0,0,0,0,0,0])
    # print(res3)
    # ctr_name = "joint_based_cartesian_traj_controller"
    # tc.load_controller(ctr_name)
    # tc.switch_controller(ctr_name)
    # rospy.sleep(2)
    # res2 = tc.cartesian_pos([0.1, 0, 0, 0, 0, 0])
    # print(res2)
    # ctr_name = "joint_group_vel_controller"
    # # tc.load_controller( ctr_name )
    # tc.switch_controller( ctr_name )
    # rospy.sleep(2.0)
    # tc.joint_vel([0.0, 0.0, 0., 0., 0., 0.1])
    # rospy.sleep(2.0)
    # tc.speedj( [0.0, 0.0, 0.0, 0.0, 0.0, 0.5] )
    # rospy.sleep(2.0)
    # pos0 = [-18.5, -12.12, 82.13, -62.42, 69.73, -91.85]
    # pos0 = [ i/180*3.1415926 for i in pos0 ]
    # tc.movej(pos0)
    # rospy.sleep(2.0)
    # tc.joint_vel( 6*[0.0] )
    # rospy.sleep(0.5)

def test_urm():
    urm = URM()
    urm.init_node("ur3_script_move")
    urm.init_robot()
    rospy.sleep(0.5)
    # p0 = urm.now_ur_pos
    # l = urm.now_ur_pos
    # p0 = [-0.27337581316103154, -0.17230540910829717, 1.2581291198730469, -0.9788311163531702, 1.3788626194000244, -1.581299130116598]
    p0_up = [0.2373865246772766, -1.2795637289630335, 1.305098056793213, -3.247901741658346, 4.789255142211914, -1.457167927418844]
    #[-0.028809372578756154, -1.489401642476217, 1.5588951110839844, -3.291513268147604, 4.475388050079346, -1.4557345549212855]

    # p0[5] = -1.2
    # 1. joint space control to pose
    urm.movej(p0_up)
    # res1 = urm.go_to_init_upinstall()
    # print(res1)
        # [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0])
    # 2. joint speed control   
    # urm.speedj([0.0, 0.0, 0.0, 0.0, 0.0, -0.5], 4)
    rospy.sleep(2.0)
    #
    # 3. TCP catersian speed control
    urm.speedl( [-0.01, 0., 0, -0., 0.0, 0 ], 2)
    rospy.sleep(2.0)
    # 4. TCP catersian position control
    # urm.movel( [ 0.1, 0, 0,  0 ,0, 0  ] )
    # print(urm.now_ur_pos)
    # print(urm.now_vel)
    # urm.relative_movel([0,-0.1,0])

from hybridCtr import HybridController as HBCTR
from FT300_kf import FTKF, FTKFListener
import numpy as np
import warnings
warnings.simplefilter('ignore', ResourceWarning)
from UtilTrans import *
from URutility import *
import threading

class URDemo:

    def __init__(self):
        self.urm = URM()
        self.HBctr = HBCTR()
        self.frequency = 50

    def init(self):
        self.urm.init_node("ur3_script_move")
        self.urm.init_robot()
        self.ft_kf = FTKFListener()
        self.rate = rospy.Rate(self.frequency)
        self.vd_pub = Pub("V_d", "twist")

    def go_to_jpose(self, p):
        res = self.urm.movej(p)

    def get_FT_wrench(self):
        return self.ft_kf.f

    def get_jpose(self):
        q = self.urm.now_ur_pos
        return q

    def get_ee_p_rot(self):
        position, quaternion = self.get_tf_pose_rot()
        rot = quaternion2rot(quaternion)
        inv_rot = quaternion2invRot(quaternion)
        angle = quaternion.angle
        return position,rot, inv_rot, angle

    def rate_sleep(self):
        self.rate.sleep()

def test_ur_move():
    print("====  test ur_move ======")
    p0_init = [-0.04932, -1.43655, 1.52059, -3.223, 4.50148, -1.46332]
    p0_touch_unface = [-0.10995, -1.15815,
                       0.88386, -2.07841, 4.60772, -1.57329]
    p_FT_x_down = [1.57077, -1.57072, 1.57086, -1.57077, 6.28311, -0.02446]
    p_FT_y_down = [1.57077, -1.57075, 1.57077, -1.57088, 6.28319, -1.57334]
    p_FT_z_down = [1.57077, -1.57074, 1.57083, -1.57081, 4.71566, -1.57332]
    p_FT_x_up = [1.57077, -1.57083, 1.5708, -3.14158, 4.71567, -1.57325]
    ur = URDemo()
    ur.init()
    sleep(0.5)
    # res1 = ur.go_to_jpose(p0_init)
    res = ur.go_to_jpose(p_FT_x_up)
    # res = ur.go_to_jpose(p_FT_y_down)
    sleep(3)
    # F_0 = ur.get_FT_wrench()
    # trace_arg(F_0)

def test_FT_calibration():
    print("====  test FT calibration ======")
    p0_init = [-0.04932, -1.43655, 1.52059, -3.223, 4.50148, -1.46332]
    p_FT_x_down = [1.57077, -1.57072, 1.57086, -1.57077, 6.28311, -0.02446]
    p_FT_y_down = [1.57077, -1.57075, 1.57077, -1.57088, 6.28319, -1.57334]
    p_FT_z_down = [1.57077, -1.57074, 1.57083, -1.57081, 4.71566, -1.57332]
    ur = URDemo()
    ur.init()
    sleep(0.5)
    res1 = ur.go_to_jpose(p_FT_x_down)
    sleep(12)
    res3 = ur.go_to_jpose(p_FT_z_down)
    sleep(5)
    res2 = ur.go_to_jpose(p_FT_y_down)
    sleep(12)
    res3 = ur.go_to_jpose(p_FT_z_down)
    sleep(10)
    # F_0 = ur.get_FT_wrench()
    # trace_arg(F_0)

def test_ori_ctr():
    print("====  test ori controller ======")
    p0_touch_face = [-0.12623, -1.12464, 0.90863, -2.24747, 4.75904, -1.63225]
    p0_init = [-0.04932, -1.43655, 1.52059, -3.223, 4.50148, -1.46332]
    p0_touch_unface = [-0.10995, -1.15815,
                       0.88386, -2.07841, 4.60772, -1.57329] # left up
    p0_untouch_face = [-0.15507, -1.24976,
                       1.13147, -2.34994, 4.77766, -1.69393]
    p0_touch_unface = [-0.04143, -1.21044,
                       1.05306, -2.36349, 4.41011, -1.48096] # left down
    p0_touch_unface = [-0.21524, -1.23101,
                       0.88651, -1.90119, 5.15509, -1.66937] # right up
    p0_touch_unface = [-0.21524, -1.23101,
                       0.88651, -1.90119, 5.15509, -1.66937]  # right up
    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(3)

    # 1. go to experiment init position
    res1 = ur.go_to_jpose(p0_touch_unface)
    sleep(4)

    # 2. get force data
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    # print(F_0)
    # sys.exit(0)
    # 3. ori control
    # delta_F_threshold_1 = [8, 10, 18, 0.05, 0.05, 10]
    # F_d = [0, 0, -25, 0, 0, 0]
    # K_ori = np.array([1e8, 1e8, 1e8, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    # trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
    # delta_X_in_FT = np.insert(omega_d, 0, [0, 0, 0])
    HZ = 50

    loop = 0
    # while np.sum(abs_delta_F < delta_F_threshold_1) < 6:
    #     trace_arg("stage 1")
    #     trace_loop(loop)
    #     try:
    #         delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
    #         F_0 = ur.get_FT_wrench()
    #         v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    #         trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
    #         loop += 1
    #         ur.rate_sleep()
    #     except KeyboardInterrupt:
    #         rospy.signal_shutdown("KeyboardInterrupt")
    #         raise
    # print("finished...")

    delta_F_threshold_2 = [3, 10, 1, 0.01, 0.01, 2]
    F_t = [0, 0, -29, 0, 0, 0]
    F_comp = [0, 0, 0, 0.168, -0.045, 0]
    F_d = list_minus(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    loop = 0
    while np.sum(abs_delta_F < delta_F_threshold_2) < 6:
        trace_arg("stage 2")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
    print("finished...")

def test_ori_dyn():
    print("====  test ori controller ======")
    p0_init = [-0.04932, -1.43655, 1.52059, -3.223, 4.50148, -1.46332]
    p0_touch_unface = [-0.21524, -1.23101, 0.88651, -
                    1.90119, 5.15509, -1.66937]  # right up
    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(3)

    # 1. go to experiment init position
    res1 = ur.go_to_jpose(p0_touch_unface)
    sleep(4)

    # 2. get force data
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    # print(F_0)
    # sys.exit(0)
    # 3. ori control
    delta_F_threshold_1 = [8, 10, 18, 0.05, 0.02, 10]
    F_t = [0, 0, -29, 0, 0, 0]
    # F_comp = [0, 0, 0, 0., -0.0, 0]
    F_comp = [0, 0, 0, 0.168, -0.045, 0]
    # F_comp = [0, 0, 0, 0.168/2, -0.045/2, 0]
    F_d = list_minus(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 1e8, 1, 1, 1e8])
    v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
    # delta_X_in_FT = np.insert(omega_d, 0, [0, 0, 0])
    HZ = 50

    loop = 0
    while np.sum(abs_delta_F < delta_F_threshold_1) < 6:
        trace_arg("stage 1")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished...")

    delta_F_threshold_2 = [3, 10, 2, 0.2, 0.1, 2]
    F_t = [0, 0, -29, 0, 0, 0]
    F_comp = [0, 0, 0, 0.168, -0.045, 0]
    F_d = list_minus(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    loop = 0
    while np.sum(abs_delta_F < delta_F_threshold_2) < 6:
        trace_arg("stage 2")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished...")


def test_ori_ctr_stage_1_2():
    print("====  test ori controller ======")
    p0_touch_face = [-0.12623, -1.12464, 0.90863, -2.24747, 4.75904, -1.63225]
    p0_init = [0.20053, -2.02754, 1.69492, -2.70726, 4.85173, -1.70402]
    p0_touch_unface = [-0.10995, -1.15815,
                       0.88386, -2.07841, 4.60772, -1.57329]
    p0_untouch_face = [-0.15507, -1.24976,
                       1.13147, -2.34994, 4.77766, -1.69393]
    p0_touch_unface = [-0.04143, -1.21044,
                       1.05306, -2.36349, 4.41011, -1.48096]
    p0_touch_unface = [-0.21524, -1.23101, 0.88651, -1.90119, 5.15509, -1.66937]  # right up
    p0_touch_unface = [-0.19083, -1.32135, 1.23862, -2.6019, 5.11803, -1.58217] # right down
    p0_touch_unface = [-0.06702, -1.12979,
                       0.85873, -2.34868, 4.34746, -1.60288]
  # left down
    # p0_touch_unface = [-0.07496, -1.00436, 0.63303, -2.19539, 4.35075, -1.52971] # large force -5-8N left down
    p0_touch_unface = [0.71431, -1.40066, 1.18128, -2.55281, 3.96198, -1.45714]# window left up no left rotate touch, will slide on the window
    p0_touch_unface = [0.692, -0.9567, 0.19264, -1.97871, 3.92475, -1.64505]
 # window left up left rotate touch , large incline angle in the Z axis 
    p0_touch_unface = [0.34265, -1.49027, 1.35336, -2.80789, 4.8922, -1.50369] # right up touch
    p0_touch_unface = [0.61002, -1.13424, 1.03547, -3.08636, 3.99281, -1.43158]
 # left down window
    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(3)

    # 1. go to experiment init position
    res1 = ur.go_to_jpose(p0_touch_unface)
    sleep(4)

    # 2. get force data
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    # print(F_0)
    # sys.exit(0)
    # 3. ori control
    delta_F_threshold_1 = [8, 10, 2, 0.08, 0.08, 0.3] # 0.08, 0.08 for steel plate
    F_t = [0, 0, -2, 0, 0, 0]
    # F_comp = [0, 0, 0, 0., -0.0, 0]
    F_comp = [0.00, 0, 0, -0.006, 0.207, -0]
    # F_comp = [0, 0, 0, 0.168/2, -0.045/2, 0]
    F_d = list_add(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 2000, 1, 2, 1e8])  # 1,2 for steel plate
    v_d,abs_delta_F = ur.HBctr.ori_ctr(F_0,F_d, K_ori)
    trace_arg("abs_delta_F",abs_delta_F, "v_d", v_d)
    # delta_X_in_FT = np.insert(omega_d, 0, [0, 0, 0])
    HZ = 50
    limit_V = np.array([0, 0, 0.1, 0.05, 0.05, 0.05])
    loop = 0
    while np.sum(abs_delta_F[2:] < delta_F_threshold_1[2:]) < 4:
        trace_arg("stage 1")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished stage 1...")

    delta_F_threshold_2 = [3, 10, 1, 0.08, 0.08, 2]
    F_t = [0, 0, -13, 0, 0, 0]
    F_comp = [0.00, 0, 0, -0.006, 0, -0]
    F_d = list_add(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    # v_d = V_d_limitation(v_d)
    limit_V = np.array([0, 0, 0.1, 0.03, 0.03, 0.1])
    loop = 0
    while np.sum(abs_delta_F[2:-1] < delta_F_threshold_2[2:-1]) < 3:
        trace_arg("stage 2")
        trace_loop(loop)
        try:
            
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished...")


def test_ori_ctr_stage_2():
    print("====  test ori controller ======")
    p0_touch_face = [-0.12623, -1.12464, 0.90863, -2.24747, 4.75904, -1.63225]
    p0_init = [0.20053, -2.02754, 1.69492, -2.70726, 4.85173, -1.70402]
    p0_touch_unface = [-0.10995, -1.15815,
                       0.88386, -2.07841, 4.60772, -1.57329]
    p0_untouch_face = [-0.15507, -1.24976,
                       1.13147, -2.34994, 4.77766, -1.69393]
    p0_touch_unface = [-0.04143, -1.21044,
                       1.05306, -2.36349, 4.41011, -1.48096]
    p0_touch_unface = [-0.21524, -1.23101, 0.88651, -
                       1.90119, 5.15509, -1.66937]  # right up
    p0_touch_unface = [-0.19083, -1.32135, 1.23862, -
                       2.6019, 5.11803, -1.58217]  # right down
    p0_touch_unface = [-0.06702, -1.12979,
                       0.85873, -2.34868, 4.34746, -1.60288]
  # left down
    # p0_touch_unface = [-0.07496, -1.00436, 0.63303, -2.19539, 4.35075, -1.52971] # large force -5-8N left down
    # window left up no left rotate touch, will slide on the window
    p0_touch_unface = [0.71431, -1.40066, 1.18128, -2.55281, 3.96198, -1.45714]
    # p0_touch_unface = [0.692, -0.9567, 0.19264, -1.97871, 3.92475, -1.64505]
 # window left up left rotate touch , large incline angle in the Z axis
    # p0_touch_unface = [0.34265, -1.49027, 1.35336, -2.80789, 4.8922, -1.50369]  # right up touch
    # p0_touch_unface = [0.61002, -1.13424, 1.03547, -3.08636, 3.99281, -1.43158]
 # left down window
    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(3)

    # 1. go to experiment init position
    res1 = ur.go_to_jpose(p0_touch_unface)
    sleep(4)

    # 2. get force data
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    # print(F_0)
    # sys.exit(0)
    # 3. ori control
    # 0.08, 0.08 for steel plate
    # delta_F_threshold_1 = [8, 10, 2, 0.08, 0.08, 0.3]
    # F_t = [0, 0, -2, 0, 0, 0]
    # # F_comp = [0, 0, 0, 0., -0.0, 0]
    # F_comp = [0.00, 0, 0, -0.006, 0.207, -0]
    # # F_comp = [0, 0, 0, 0.168/2, -0.045/2, 0]
    # F_d = list_add(F_t, F_comp)
    # K_ori = np.array([1e8, 1e8, 2000, 1, 2, 1e8])  # 1,2 for steel plate
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    # trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
    # # delta_X_in_FT = np.insert(omega_d, 0, [0, 0, 0])
    # limit_V = np.array([0, 0, 0.1, 0.05, 0.05, 0.05])
    # loop = 0
    # while np.sum(abs_delta_F[2:] < delta_F_threshold_1[2:]) < 4:
    #     trace_arg("stage 1")
    #     trace_loop(loop)
    #     try:
    #         delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
    #         F_0 = ur.get_FT_wrench()
    #         v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    #         v_d = V_d_limitation(v_d, limit_V)
    #         trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
    #         loop += 1
    #         ur.rate_sleep()
    #     except KeyboardInterrupt:
    #         rospy.signal_shutdown("KeyboardInterrupt")
    #         raise
    #     exp_publish(ur.vd_pub, v_d)
    # print("finished stage 1...")

    delta_F_threshold_2 = [3, 10, 1, 0.08, 0.08, 2]
    F_t = [0, 0, -13, 0, 0, 0]
    F_comp = [0.00, 0, 0, -0.006, 0, -0]
    F_d = list_add(F_t, F_comp)
    K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    # v_d = V_d_limitation(v_d)
    limit_V = np.array([0, 0, 0.1, 0.03, 0.03, 0.1])
    loop = 0
    while np.sum(abs_delta_F[2:-1] < delta_F_threshold_2[2:-1]) < 3:
        trace_arg("stage 2")
        trace_loop(loop)
        try:

            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished...")


def test_closing_touch_linear_clean():
    print("====  test_closing_touch_linear_clean ======")

    p0_init = [0.20053, -2.02754, 1.69492, -2.70726, 4.85173, -1.70402]

    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(5)

    # 1. touch the surface
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_1 = [3.0, 10.0, 2.0, 0.08, 0.08, 2]
    F_t = [0, 0, -2, 0, 0, 0]
    F_comp = [0.00, 0, 0.0, -0.006, 0, -0]
    F_d = list_add(F_t, F_comp)
    v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
    # res1 = ur.go_to_jpose(p0_touch_unface)
    limit_V = np.array([0, 0, 0.02, 0.03, 0.03, 0.1])
    loop = 0

    print("abs_delta_F[2]", abs_delta_F[2])
    print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
    while abs_delta_F[2] >= delta_F_threshold_1[2]:
        print("abs_delta_F[2]", abs_delta_F[2])
        print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
        trace_arg("=====stage 1=====")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished touch the window stage 1...")
    sleep(2)

    #2 . adjust z orientation
    q = ur.get_jpose()
    qd = ur.HBctr.z_ori_adjustment(q)
    ur.go_to_jpose(qd)
    print("finished z orientation adjustment stage 2...")
    sleep(1)

    # 3. orientation control for fully touching
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_2 = [3.0, 10.0, 1.0, 0.08, 0.08, 2.0]
    F_t = [.0, .0, -13.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    # K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    # v_d = V_d_limitation(v_d)
    limit_V = np.array([0, 0, 0.1, 0.03, 0.03, 0.1])
    loop = 0
    
    while np.sum(abs_delta_F[2:-1] < delta_F_threshold_2[2:-1]) < 3:
        trace_arg("====stage 3====")
        trace_loop(loop)
        try:

            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished xy and Fz adjustment for closing touch in stage 3...")

    # 4. linear cleaning, 0.01m/s speed running 4 second
    clean_line_loop = 0
    clean_loop_threshold = 3
    
    while clean_line_loop <= 3:
        clean_line_loop += 1

        clean_speed = -0.02
        Duration = 4.0
        v_w = [clean_speed*((clean_line_loop%2-0.5)*2), 0.0]
        limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
        F_t = [.0, .0, -13.0, .0, .0, .0]
        F_comp = [0.00, .0, .0, -0.006, .0, -.0]
        F_d = list_add(F_t, F_comp)
        window_cleaning_movement(ur, v_w, Duration, F_d)
        # print("finished cleaning in one line in stage 4...")
        print("finished cleaning in one line in stage 4, with cleaning loop %s..." %(clean_line_loop))

        # 5. linear change position, 0.01m/s speed running 1 second
        if clean_line_loop <= clean_loop_threshold:
            clean_speed = 0.02
            Duration = 4.0
            v_w = [0.0, clean_speed]
            limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
            F_t = [.0, .0, -13.0, .0, .0, .0]
            F_comp = [0.00, .0, .0, -0.006, .0, -.0]
            F_d = list_add(F_t, F_comp)
            window_cleaning_movement(ur, v_w, Duration, F_d)
            # print("finished cleaning in one line in stage 4...")
            print("finished change cleaning position to next in stage 4, with change line to line %s..." %(clean_line_loop+1))
        
    print("finished change cleaning position to next in stage 5...")

    res1 = ur.go_to_jpose(p0_init)
    sleep(3)
    print("finish cleaning and back to init position stage 6...")


def test_closing_touch_circular_clean():
    print("====  test_closing_touch_linear_clean ======")

    p0_init = [0.20053, -2.02754, 1.69492, -2.70726, 4.85173, -1.70402]

    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(5)

    # 1. touch the surface
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_1 = [3.0, 10.0, 2.0, 0.08, 0.08, 2]
    F_t = [0, 0, -2, 0, 0, 0]
    F_comp = [0.00, 0, 0.0, -0.006, 0, -0]
    F_d = list_add(F_t, F_comp)
    v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
    # res1 = ur.go_to_jpose(p0_touch_unface)
    limit_V = np.array([0, 0, 0.02, 0.03, 0.03, 0.1])
    loop = 0

    print("abs_delta_F[2]", abs_delta_F[2])
    print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
    while abs_delta_F[2] >= delta_F_threshold_1[2]:
        print("abs_delta_F[2]", abs_delta_F[2])
        print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
        trace_arg("=====stage 1=====")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished touch the window stage 1...")
    sleep(2)

    #2 . adjust z orientation
    q = ur.get_jpose()
    qd = ur.HBctr.z_ori_adjustment(q)
    ur.go_to_jpose(qd)
    print("finished z orientation adjustment stage 2...")
    sleep(1)

    # 3. orientation control for fully touching
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_2 = [3.0, 10.0, 1.0, 0.08, 0.08, 2.0]
    F_t = [.0, .0, -13.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    # K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    # v_d = V_d_limitation(v_d)
    limit_V = np.array([0, 0, 0.1, 0.03, 0.03, 0.1])
    loop = 0

    while np.sum(abs_delta_F[2:-1] < delta_F_threshold_2[2:-1]) < 3:
        trace_arg("====stage 3====")
        trace_loop(loop)
        try:

            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished xy and Fz adjustment for closing touch in stage 3...")

    # 4. linear cleaning, 0.01m/s speed running 4 second

    # clean_line_loop += 1
    loop = 0
    HZ = 50 
    clean_speed = -0.02
    Duration = 4.0
    # v_w = [clean_speed*((clean_line_loop % 2-0.5)*2), 0.0]
    limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
    F_t = [.0, .0, -13.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    window_circle_cleaning_movement(ur,F_d,0.12)
    # print("finished cleaning in one line in stage 4...")
    # print("finished cleaning in one line in stage 4, with cleaning loop %s..." % (
    #     clean_line_loop))


    print("finished change cleaning position to next in stage 5...")



def test_linear_clean():
    print("====  test linear_clean ======")
    p0_init = [0.20053, -2.02754, 1.69492, -2.70726, 4.85173, -1.70402]
    p0_touch_face = [0.28427, -1.29416, 1.39266, -3.22424, 4.78619, -1.56541]
    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(3)
    # 1. go to experiment init position
    res1 = ur.go_to_jpose(p0_touch_face)
    sleep(4)
    # 2. linear cleaning

    clean_speed = 0.02
    Duration = 4.0
    v_w = [clean_speed, 0.0]
    limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
    F_t = [.0, .0, -13.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    window_cleaning_movement(ur, v_w, Duration, F_d)
    print("finished cleaning in one line in stage 4...")

def window_cleaning_movement(ur, v_w, Duration, F_d, limit_V=np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])):
    HZ = 50
    loop = 0
    # clean_speed = 0.02
    # Duration = 4.0
    t = 0.0
    # limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
    # F_t = [.0, .0, -13.0, .0, .0, .0]
    # F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    # F_d = list_add(F_t, F_comp)
    while t < Duration:
        trace_arg("==== cleaning the window with velocity, ","vertical", v_w[0], " horizontal:",v_w[1], "====")
        trace_loop(loop)
        trace_arg("duration", t)
        try:
            F_0 = ur.get_FT_wrench()
            # v_w = [clean_speed, 0.0]  # x: vertical, y: horizontal
            v_d, abs_delta_F = ur.HBctr.linear_clean_ctr(F_0, F_d, v_w)
            v_d = V_d_limitation(v_d, limit_V)            
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            loop += 1
            t = loop * 1.0 / HZ
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished cleaning in one line in stage 4...")


def window_circle_cleaning_movement(ur, F_d, C_R=0.12, limit_V=np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])):
    HZ = 50
    loop = 0
    # clean_speed = 0.02
    
    t = 0.0
    # limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
    # F_t = [.0, .0, -13.0, .0, .0, .0]
    # F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    # F_d = list_add(F_t, F_comp)
    C_omega = 2*math.pi/20.0 #  if C_omega is bigger to 2pi/10, the trajectory will be rectangle
    Duration = 20.0* 3
    while t < Duration:
        trace_loop(loop)
        t = loop * 1.0 / HZ
        trace_arg("duration", t)
        v_r = circle_V(t,C_omega,C_R)
        v_w = [-v_r[0], -v_r[1]]
        trace_arg("==== cleaning the window with velocity, ",
                  "vertical", v_w[0], " horizontal:", v_w[1], "====")       

        try:
            F_0 = ur.get_FT_wrench()
            # v_w = [clean_speed, 0.0]  # x: vertical, y: horizontal
            v_d, abs_delta_F = ur.HBctr.linear_clean_ctr(F_0, F_d, v_w)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            loop += 1
            
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished cleaning in one line in stage 4...")


def test_downinstall_closing_touch_circular_clean():
    print("====  test_closing_touch_linear_clean ======")

    p0_init = [-0.51112, -1.19106, 2.26829, -0.98499, 0.79212, -1.87839]

    ur = URDemo()
    ur.init()

    sleep(0.5)
    res1 = ur.go_to_jpose(p0_init)
    sleep(5)

    # 1. touch the surface
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_1 = [3.0, 10.0, 2.0, 0.08, 0.08, 2]
    F_t = [0, 0, -2, 0, 0, 0]
    F_comp = [0.00, 0, 0.0, -0.006, 0, -0]
    F_d = list_add(F_t, F_comp)
    v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
    # res1 = ur.go_to_jpose(p0_touch_unface)
    limit_V = np.array([0, 0, 0.02, 0.03, 0.03, 0.1])
    loop = 0

    print("abs_delta_F[2]", abs_delta_F[2])
    print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
    while abs_delta_F[2] >= delta_F_threshold_1[2]:
        print("abs_delta_F[2]", abs_delta_F[2])
        print("delta_F_threshold_1[2]", delta_F_threshold_1[2])
        trace_arg("=====stage 1=====")
        trace_loop(loop)
        try:
            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.force_touch_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished touch the window stage 1...")
    sleep(2)

    #2 . adjust z orientation
    q = ur.get_jpose()
    qd = ur.HBctr.z_ori_adjustment(q)
    ur.go_to_jpose(qd)
    print("finished z orientation adjustment stage 2...")
    sleep(1)

    # 3. orientation control for fully touching
    F_0 = ur.get_FT_wrench()
    trace_arg(F_0)
    HZ = 50
    delta_F_threshold_2 = [3.0, 10.0, 1.0, 0.08, 0.08, 2.0]
    F_t = [.0, .0, -10.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    # K_ori = np.array([1e8, 1e8, 3000, 5, 5, 1e8])
    # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
    v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
    # v_d = V_d_limitation(v_d)
    limit_V = np.array([0, 0, 0.1, 0.03, 0.03, 0.1])
    loop = 0

    while np.sum(abs_delta_F[2:-1] < delta_F_threshold_2[2:-1]) < 3:
        trace_arg("====stage 3====")
        trace_loop(loop)
        try:

            delta_X_in_base = ur.urm.speedl_tool(v_d, 1, 1/HZ)
            F_0 = ur.get_FT_wrench()
            # v_d, abs_delta_F = ur.HBctr.ori_ctr(F_0, F_d, K_ori)
            v_d, abs_delta_F = ur.HBctr.ori_pid_ctr(F_0, F_d)
            v_d = V_d_limitation(v_d, limit_V)
            trace_arg("abs_delta_F", abs_delta_F, "v_d", v_d)
            loop += 1
            ur.rate_sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        exp_publish(ur.vd_pub, v_d)
    print("finished xy and Fz adjustment for closing touch in stage 3...")

    # 4. linear cleaning, 0.01m/s speed running 4 second

    # clean_line_loop += 1
    loop = 0
    HZ = 50
    clean_speed = -0.02
    Duration = 4.0
    # v_w = [clean_speed*((clean_line_loop % 2-0.5)*2), 0.0]
    limit_V = np.array([0.04, 0.04, 0.1, 0.03, 0.03, 0.1])
    F_t = [.0, .0, -13.0, .0, .0, .0]
    F_comp = [0.00, .0, .0, -0.006, .0, -.0]
    F_d = list_add(F_t, F_comp)
    window_circle_cleaning_movement(ur, F_d, 0.12)
    # print("finished cleaning in one line in stage 4...")
    # print("finished cleaning in one line in stage 4, with cleaning loop %s..." % (
    #     clean_line_loop))

    print("finished change cleaning position to next in stage 5...")

def exp_define_pub():
    # define_force_pub("delta_delta_F")
    ddF_pub = Pub("delta_delta_F", "force")
    # define_force_pub("delta_F_real")
    dF_pub = Pub("delta_F_real", "force")  
    # define_twist_pub("delta_X_in_FT")
    ee_in_ee_TW_pub = Pub("delta_X_in_FT", "twist")
    # define_twist_pub("delta_X_in_base")
    ee_in_base_TW_pub = Pub("delta_X_in_base", "twist")
    return ddF_pub, dF_pub, ee_in_ee_TW_pub, ee_in_base_TW_pub


if __name__ == '__main__':
    # test_joint_pose_move()
    # test_urm()
    # test_FT_calibration()
    # test_ur_move()
    # test_ori_ctr()
    # test_ori_ctr_stage_1_2()
    # test_ori_ctr_stage_2()  #  useful !! latest
    # test_linear_clean()
    # test_closing_touch_linear_clean()
    # test_closing_touch_circular_clean()\
    test_downinstall_closing_touch_circular_clean()
