#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-02-26 03:00:58
#LastEditTime: 2022-02-26 03:01:35
#LastEditors: Guangli
#Description: ur3 cleaning state and its control process
#FilePath: /URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/ur_state.py
##############

import rospy

from abc import abstractmethod


class CTRstate:

    def __init__(self, name) :
        self.name = name
        self.state = 0

    def set_state_status(self, status):
        self.status = status

    def print_state(self):
        if self.status == 1:
            print("UR now implement processing %s" %self.name)
        elif self.status == 0:
            print("Process %s is finished!" %self.name)

    @classmethod
    def init_param(self):
        pass

    @abstractmethod
    def update():
        pass

    @abstractmethod
    def check_condition(self):
        pass

    def spin(self):
        self.init_param()
        self.set_state_status(1)
        while self.check_condition():
            self.print_state()
            try:
                self.update()
            except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        # print("finish process %s" %self.name)
        self.set_state_status(0)
        self.print_state()


class S_LinearCleaning(CTRstate):

    def __init__(self) -> None:
        super().__init__()