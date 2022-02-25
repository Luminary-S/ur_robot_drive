#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-05 11:04:25
#LastEditTime: 2022-01-05 11:04:26
#LastEditors: Guangli
#Description: 
#FilePath: /src/URpacks/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/filter.py
##############
# import sophus as sp
import numpy as np
from pykalman import KalmanFilter as KF

class Filter(object):

    def __init__(self):
        # super().__init__()
        # self.past_val = 0.0
        self.buf = []
        self._BUFF_SIZE = 10
        self.filter_buf = []
        self.sum = 0.0
        self.past = 0.0

    def add(self, data):
        if len(self.buf) <  self._BUFF_SIZE:
            self.buf.append(data)
        else:
            self.buf.pop(0)
            self.buf.append(data)
        return len(self.buf)

    def get_latest(self):
        return self.buf[-1]

    def get_filtered_latest(self):
        return self.filter_buf[-1]

    def get_filtered_init(self,num=10):
        return np.average(np.array(self.filter_buf[-num:]), axis=0).tolist()
    
    def get_init(self, num=10):
        # get average of num of data as initial data
        return np.average(np.array(self.buf[-num:]) , axis=0).tolist()
    
    def set_past(self, val):
        self.past = val

    def IIR(self, now, k):
        # len_buf = len(self.buf)
        past = self.past
        res = ( now + (k-1)*past ) * 1.0 / k + 0.5
        self.set_past(now)
        self.filter_buf.append(res)
        return res

    def average(self, now, window=10):
        len_buf = len(self.buf)
        if len_buf >= window:
            self.sum = self.sum - self.buf[-window]
        self.sum = self.sum + now
        res = self.sum*1.0/window
        self.filter_buf.append(res)
        return res
        # for i in range(number):

    #NOTE:window should be odd
    def median(self, now, window=5):
        len_buf = len(self.buf)
        m_id = window//2+1
        if len_buf >= window:
            temp_list = self.buf[(-window):]
            temp_list.sort()
            res = temp_list[m_id]
            self.filter_buf.append(res)
            return res
        self.filter_buf.append(now)
        return now

    #function: Kalman filter part
    def kf_define(self, Transition_Matrix, Observation_Matrix, initstate, initcovariance, transistionCov, observationCov):
        import inspect
        self.kf = KF(transition_matrices=Transition_Matrix,
                     observation_matrices=Observation_Matrix,
                     initial_state_mean=initstate,
                     initial_state_covariance=initcovariance,
                     transition_covariance=transistionCov,
                     observation_covariance=observationCov)
        # print(inspect.getfullargspec(self.kf.__init__)[0])
    # def kf_set_init(self, x0=np.zeros(6), cov0=np.eye(6)):
    #     self.X0 = x0
    #     self.cov0 = cov0
    #     self.kf.
        # print(self.X0,self.cov0)

    def kf_update(self, X,P,M ):        
        X, P = self.kf.filter_update( X, P, M)
        self.filter_buf.append(X)
        return X,P


def sim():
    import random
    import matplotlib.pyplot as plt
    # data = random.random(10)
    a = [1,2,3,4,5,6,7,8,9,10,11,12]
    random.shuffle(a)
    print(a)

    f = Filter()
    avg_buf = []
    med_buf = []
    IIR_buf = []
    for i in a:
        f.add(i)
        avg = f.average(i, 5)
        print("average:", avg)
        avg_buf.append(avg)
        med = f.median(i, 5)
        print("median:",med)
        med_buf.append(med)
        iir = f.IIR(i, 5)
        print("IIR:",iir)
        IIR_buf.append(iir)

    fig = plt.figure()
    plt.plot(a, label="origin")
    plt.plot(avg_buf, label="average")
    plt.plot(med_buf, label="median")
    plt.plot(IIR_buf, label="IIR")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    sim()
    # sim_kf()
    # test()