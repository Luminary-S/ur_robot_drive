#!/usr/bin/python
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-01 00:24:22
#LastEditTime: 2022-01-01 00:24:23
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/URClient.py
##############

from os import name
import sys,rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointTolerance)
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from std_srvs.srv import Trigger, TriggerRequest
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    FollowCartesianTrajectoryResult,
    CartesianTrajectoryPoint)
from controller_manager_msgs.srv import (
    SwitchControllerRequest, 
    SwitchController, 
    LoadControllerRequest, 
    LoadController)

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class URClient(object):

    def __init__(self,name):
        # super(URClient, self).__init__(*args)
        self.init_robot(name)
        print("init urclient")
        # pass

    def init_robot(self, name="ur_robot_driver_integration_test"):
        """Make sure the robot is booted and ready to receive commands"""

        # rospy.init_node('ur_robot_driver_integration_test')
        rospy.init_node(name)
        rospy.loginfo("init ros node: "+ name)
        self.basic_client()

    def basic_client(self):
        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller',
                                             SwitchController)
        self.load_srv = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        # self.send_program_srv = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program',Trigger)
        # try:
        #     self.send_program_srv.wait_for_service(timeout)
        # except rospy.exceptions.ROSException as err:
        #     rospy.logerr(
        #         "Could not reach resend_robot_program service. Make sure that the driver is "
        #         "actually running in headless mode."
        #         " Msg: {}".format(err))

        self.script_publisher = rospy.Publisher(
            "/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.twist_pub = rospy.Publisher(
            "/twist_controller/command", geometry_msgs.msg.Twist, queue_size=1)

    def added_client(self):
        timeout = rospy.Duration(10)

        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.trajectory_client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        try:
            self.trajectory_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.cartesian_passthrough_trajectory_client = actionlib.SimpleActionClient(
            'forward_cartesian_trajectory', FollowCartesianTrajectoryAction)
        try:
            self.cartesian_passthrough_trajectory_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.joint_passthrough_trajectory_client = actionlib.SimpleActionClient(
            'forward_joint_trajectory', FollowJointTrajectoryAction)
        try:    
            self.joint_passthrough_trajectory_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach joint passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.cartesian_trajectory_client = actionlib.SimpleActionClient(
            'follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        try:
            self.cartesian_trajectory_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach cartesian controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        #NOTE： if IO state added, io_client can be used
        self.set_io_client = rospy.ServiceProxy(
            '/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach SetIO service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

    def send_cartesian_trajectory(self):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(
                self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # The following list are arbitrary positions
        # Change to your own needs if desired
        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    0.4, -0.1, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    0.4, 0.3, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    0.4, 0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
            ),
        ]
        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code))

    ###############################################################################################
    #                                                                                             #
    # Methods defined below are for the sake of safety / flexibility of this demo script only.    #
    # If you just want to copy the relevant parts to make your own motion script you don't have   #
    # to use / copy all the functions below.                                                       #
    #                                                                                             #
    ###############################################################################################

    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        rospy.logwarn(
            "The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo(
                    "Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == "y"
        if not confirmed:
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def choose_controller(self):
        """Ask the user to select the desired controller from the available list."""
        rospy.loginfo("Available trajectory controllers:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (joint-based): {}".format(index, name))
        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (Cartesian): {}".format(
                index + len(JOINT_TRAJECTORY_CONTROLLERS), name))
        choice = -1
        while choice < 0:
            input_str = input(
                "Please choose a controller by entering its number (Enter '0' if "
                "you are unsure / don't care): "
            )
            try:
                choice = int(input_str)
                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
                    CARTESIAN_TRAJECTORY_CONTROLLERS
                ):
                    rospy.loginfo(
                        "{} not inside the list of options. "
                        "Please enter a valid index from the list above.".format(
                            choice)
                    )
                    choice = -1
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice - len(JOINT_TRAJECTORY_CONTROLLERS)
        ]
        return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        # other_controllers.remove(target_controller)

        # srv = LoadControllerRequest()
        # srv.name = target_controller
        # try:
        # a = self.load_srv(srv)
        # except rospy.exceptions.ROSException as err:
        # rospy.logerr(
        #     "load service error."
        #     " Msg: {}".format(a))
        # print(a)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        # srv.start_controllers = []
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    '''
    function name: load_controller(target_controller)
    brief: load the controller defined in the controller.yaml, but not presetting in the launch file
    description: if setting in the launch file, reload the controller will have an error msg
    param {*} self
    param {*} target_controller
    return {*}
    example: 
    '''    
    def load_controller(self, target_controller):
        srv = LoadControllerRequest()
        srv.name = target_controller
        try:
            self.load_srv(srv)
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "load service error."
                " Msg: {}".format(err))
        # print(a)
