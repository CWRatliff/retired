'''
Extended Kalman Filter by Atsushi Sakai

with state vector {x, y, speed, heading}
'''
import math
import numpy as np
import matplotlib.pyplot as plt

spdfactor = .005
lonfeet = 82.0
latfeet = 100.0

DT = 1
t0 = 0;
xEst = np.zeros((4,1))                        # state vector
pEst = np.zeros((4,4))                        # state covariance matrix
Q = np.diag([1.0, 1.0])**2                    # process noise covariance  
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2 # measurement noise covariance

# predict new position based on dead reconning
# x - state vector input output
# u - [speed, steer]
def motion_model(x, u):
    print("motion x", x)
    print("motion u", u)
    F = np.array([[1.0, 0, 0, 0],             # state transition matrix
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 1.0]])
    B = np.array([[DT * math.cos(u[0, 1]), 0],
                  [DT * math.sin(u[0, 1]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u.T)

    print("motion ret x", x)
    return x
    
# x - state vector
# z - state vector with observations
def observation_model(x):
    H = np.array([                              # measurement function
        [1, 0, 0, 0],
        [0, 1, 0, 0]])
    z = H.dot(x)
    print("obs_mod z", z)
    return z

#motion model Jacobian matrix
# x - state vector
# u - input vector
def jacobiF(u):
    v = u[0, 0]
    d = DT * v
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
def jacobiH():
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]])
    return jH

# xEst - state vector
# pEst - state transition matrix
# z - GPS input
# u speed/steering input
def Kalman_filter(xEst, pEst, z, u):
    # predict
    xPred = motion_model(xEst, u)
    print ("xPred", xPred)
    jF = jacobiF(u)
    print("jF",jF)
    print("pEst", pEst)
    print("R", R)
    pPred = jF.dot(pEst).dot(jF.T) + R
    print("Pred", pPred)
    
    # update
    jH = jacobiH()
    zPred = observation_model(xPred)
    print("z", z)
    print("zPred", zPred)
    y = z.T - zPred
    S = jH.dot(pPred).dot(jH.T) + Q
    print("K-F S", S)
    K = pPred.dot(jH.T).dot(np.linalg.inv(S))
    xEst = xPred + K.dot(y)
    pEst = (np.eye(len(xEst)) - K.dot(jH)).dot(pPred)
    return xEst, pEst

def Kalman_start(begin):
    t0 = begin
    xEst = np.zeros((4,1))              # infinite a priori
    pEst = np.zeros((4,1))

# time - delta T of sample
# speed - fps
# hdg - radians (x-axis zero)
# x - x-axis
# y - y-axis
def Kalman_step(DT, speed, hdg, x, y):
#    u = (np.array([[speed, hdg]])).T
    u = (np.array([[speed, hdg]]))
    z = np.array([[x, y]])
    nxEst, npEst = Kalman_filter(xEst, pEst, z, u)
    return nxEst, npEst

test = [
    [1, 60, 325, 7.123, 20.345],
    [2, 60, 326, 7.146, 20.444],
    ]
for i in range(2):
    deltaT = test[i][0] - t0
    t0 = test[i][0]
    spd = test[i][1] * spdfactor
#    ang = np.deg2rad(450-test[0][2])
    ang = 3.14157/180*(450-test[i][2])
    x = -test[i][3] + lonfeet
    y = test[i][4] * latfeet

    print (spd, ang, x, y)
    xnew = Kalman_step(deltaT, spd, ang, x, y)
