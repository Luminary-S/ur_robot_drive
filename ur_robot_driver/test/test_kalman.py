#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-16 13:05:03
#LastEditTime: 2022-01-16 13:05:11
#LastEditors: Guangli
#Description: 
#FilePath: /ur_robot_driver/test/test_kalman.py
##############

class KalmanFilter:
    # '''
    # Kalman-Filter
    # 对于一个线性系统的状态差分方程为
    #     x_k = A * x_{k-1} + B*u_{k-1} + w_k
    # 其中x是系统的状态向量，大小为n*1列。A为转换矩阵，大小为n*n。
    # u为系统输入，大小为k*1。B是将输入转换为状态的矩阵，大小为n*k。
    # 随机变量w为系统噪声。注意这些矩阵的大小，它们与你实际编程密切相关。
    # 测量值当然是由系统状态变量映射出来的，方程形式如下：
    #     z_k = H*x_k + v_k
    # 注意，此处的x_k表示的是仍是上面的系统预测值，这个测量值的映射只不过是理想情况下。
    # 注意Z是测量值，大小为m*1(m取决于测量值），H也是状态变量到测量的转换矩阵。大小为m*n。随机变量v是测量噪声。
    # 首先要计算预测值、预测值和真实值之间误差协方差矩阵。
    #     (1) \hat{x_k^{-}} = A * \hat{x_{k-1}} + B * u_{k-1}
    #     (2) P_k^{-} = A * P_{k-1} * A^T + Q
    # 其中 \hat{x_k^{-}} 和 P_k^{-} 表示预测的结果和预测的协方差矩阵。
    # Q 表示系统噪声
    # 有了这两个就能计算卡尔曼增益K，再然后得到估计值，
    #     (3) K_k = P_k^{-} * H^T * (H * P_k^{-} * H^T + R)^(-1)
    #     (4) \hat{x_k} = \hat{x_k^{-}} + K_k * (z_k - H * \hat{x_k^{-}})
    # 其中 K_k 为卡尔曼增益，R 表示传感器噪声
    # 最后还要计算估计值和真实值之间的误差协方差矩阵，为下次递推做准备。
    #     (5) P_k = (I - K_k * H) * P_k^{-}
    # I 表示单位矩阵
    # '''

    def __init__(self, X0, A, B, W, H, V):
        """
        Initialise the filter
        Args:
            X: State estimate
            P: Estimate covaConfigureriance
            F: State transition model: for FT = I[6]
            Q: Process noise covariance : for FT = []
            Z: Measurement of the state X
            H: Observation model
            R: Observation noise covariance
        """
        self.X = X0
        self.A = A
        self.B = B
        self.W = W
        # self.Y = Y
        self.H = H  # state to measurement matrix m*n
        self.V = V  # m*m

    def predict(self, X, P,  U=0):
        """
        Predict the future state
        Args:
            X: State estimate
            P: Estimate covariance
        Returns:
            updated (X, P)
        """
        # Project the state ahead
        X = self.A * X + self.B * U
        P = self.F * P * (self.F.T) + self.W
        return(X, P)

    def update(self, X, P, Z):
        """
        Update the Kalman Filter from a measurement
        Args:
            X: State estimate
            P: Estimate covariance
            Z: State measurement
        Returns:
            updated (X, P)
        """
        K = P * (self.H.T) * np.inv(self.H * P * (self.H.T) + self.V)
        X += K * (Z - self.H * X)
        P = (np.identity(P.shape[1]) - K * self.H) * P
        return (X, P)
