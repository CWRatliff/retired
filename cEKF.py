'''
Adapted from Extended Kalman Filter by Atsushi Sakai
//http://atsushiasakai.github.io/PythonRobotics

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
        self.R = np.diag([0.1, 0.1, np.deg2rad(1.0), .1])**2 # measurement noise covariance

    # predict new position based on dead reconning
    # x - state vector input output
    # u - [speed, steer]
    def motion_model(self, x, u):
        print("motion x", x)
        print("motion u", u)
        F = np.array([[1.0, 0, 0, 0],             # state transition matrix
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0]])
        B = np.array([[self.DT * math.cos(x[2, 0]), 0],
                      [self.DT * math.sin(x[2, 0]), 0],
                      [0, self.DT],
                      [1.0, 0.0]])
        print("motion B mat",B)
        x = F.dot(x) + B.dot(u.T)
        print("motion x(t+1)", x)
        return x
    
    # x - state vector
    # z - state vector with observations
    def observation_model(self, x):
        H = np.array([                              # measurement function
            [1, 0, 0, 0],
            [0, 1, 0, 0]])
        z = H.dot(x)
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
            [1.0, 0.0, -dsina, self.DT * cosa],
            [0.0, 1.0, dcosa , self.DT * sina],
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
    def Kalman_update(self, z, u):
        # predict
        xPred = self.motion_model(self.xEst, u)
        jF = self.jacobiF(u)
        pPred = jF.dot(self.pEst).dot(jF.T) + self.R
        
        # update
        jH = self.jacobiH()
        zPred = self.observation_model(xPred)
        y = z.T - zPred
        S = jH.dot(pPred).dot(jH.T) + self.Q
        K = pPred.dot(jH.T).dot(np.linalg.inv(S))
        self.xEst = xPred + K.dot(y)
        self.pEst = (np.eye(len(self.xEst)) - K.dot(jH)).dot(pPred)
        return self.xEst


    # time - start time of sample
    # v - fps
    # phi - heading radians
    # x - x-axis
    # y - y-axis
    def Kalman_start(self, begin, x, y, phi, v):
        self.t0 = begin
        self.oldphi = phi
        self.xEst = np.zeros((4,1))              # infinite a priori
        self.pEst = np.zeros((4,4))
        self.xEst[0, 0] = x
        self.xEst[1, 0] = y
        self.xEst[2, 0] = phi
        self.xEst[3, 0] = v

    # time - time of sample seconds
    # v - fps
    # phi = heading in radians (E, N)
    # x - x-axis
    # y - y-axis
    def Kalman_step(self, t, x, y, phi, v):
        self.DT = t - self.t0
        self.t0 = t
        self.omega = (phi - self.oldphi) / self.DT
        u = np.array([[v, self.omega]])
        z = np.array([[x, y]])
        self.xEst = self.Kalman_update(z, u)
        return self.xEst

