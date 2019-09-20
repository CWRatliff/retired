'''
Extended Kalman Filter by Atsushi Sakai

with state vector {x, y, speed, heading}
'''
import math
import numpy as np
import matplotlib.pyplot as plt


class Kalman_filter:
    def __init__(self):
        self.DT = 1
        self.t0 = 0
        self.ang = 0
        self.xEst = np.zeros((4,1))                        # state vector
        self.pEst = np.zeros((4,4))                        # state covariance matrix
        self.Q = np.diag([1.0, 1.0])**2                    # process noise covariance  
        self.R = np.diag([0.1, 0.1, .1, np.deg2rad(1.0)])**2 # measurement noise covariance

    # predict new position based on dead reconning
    # x - state vector input output
    # u - [speed, steer]
    def motion_model(self, x, u):
        F = np.array([[1.0, 0, 0, 0],             # state transition matrix
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 1.0]])
        B = np.array([[DT * math.cos(u[0, 1]), 0],
                      [DT * math.sin(u[0, 1]), 0],
                      [DT, 0],
                      [0.0, 1.0]])

        x = F.dot(x) + B.dot(u.T)

        print("motion ret x", x)
        return x
    
    # x - state vector
    # z - state vector with observations
    def observation_model(self, x):
        H = np.array([                              # measurement function
            [1, 0, 0, 0],
            [0, 1, 0, 0]])
        z = H.dot(x)
        print("obs_mod z", z)
        return z

    #motion model Jacobian matrix
    # x - state vector
    # u - input vector
    def jacobiF(self, u):
        v = u[0, 0]
        d = self.DT * v
        alpha = u[0, 1]
        sina = math.sin(alpha)
        cosa = math.cos(alpha)
        dsina = sina * d
        dcosa = cosa * d
        jF=np.array([
            [1.0, 0.0, -dsina, DT * cosa],
            [0.0, 1.0, dcosa , DT * sina],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])
        return jF

    #observation model Jacobian matrix
    def jacobiH(self):
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]])
        return jH

    # xEst - state vector
    # pEst - state transition matrix
    # z - GPS input
    # u speed/steering input
    def Kalman_filter(self, z, u):
        # predict
        xPred = motion_model(xEst, u)
        jF = jacobiF(u)
        print("jF",jF)
        print("pEst", pEst)
        print("R", R)
        pPred = jF.dot(pEst).dot(jF.T) + R
        print("Pred", pPred)
        
        # update
        jH = jacobiH()
        zPred = observation_model(xPred)
        print("zPred", zPred)
        y = z.T - zPred
        S = jH.dot(pPred).dot(jH.T) + Q
        K = pPred.dot(jH.T).dot(np.linalg.inv(S))
        self.xEst = xPred + K.dot(y)
        self.pEst = (np.eye(len(xEst)) - K.dot(jH)).dot(pPred)


    def Kalman_start(self, begin):
        self.t0 = begin
        self.xEst = np.zeros((4,1))              # infinite a priori
        self.pEst = np.zeros((4,1))

    # time - delta T of sample
    # speed - fps
    # hdg - radians (x-axis zero)
    # x - x-axis
    # y - y-axis
    def Kalman_step(self, DelT, speed, hdg, x, y):
    #    u = (np.array([[speed, hdg]])).T
        self.DT = DelT
        u = (np.array([[speed, hdg]]))
        z = np.array([[x, y]])
        self.xEst, self.pEst = Kalman_filter(xEst, pEst, z, u)


