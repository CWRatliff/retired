'''
Extended Kalman Filter by Atsushi Sakai

with state vector {x, y, speed, heading}
'''
import math
import numpy as np
import matplotlib.pyplot as plt

DT = 1
t0 = 0;
xEst = np.zeros((4,1))                        # state vector
pEst = np.zeros((4,1))                        # state covariance matrix
Q = np.diag([1.0, 1.0])**2                    # process noise covariance  
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2 # measurement noise covariance

# predict new position based on dead reconning
def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],             # state transition matrix
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 1.0]])
    B = np.array([[DT * math.cos(steer), 0],
                  [DT * math.sin(steer), 0],
                  [0.0, DT],
                  [1.0, 0.0]])
    x = xEst
    x = F.dot(x) + B.dot(u)
    return x
    
def observation_model(x):
    H = np.array([                              # measurement function
        [1, 0, 0, 0],
        [0, 1, 0, 0]])
    z = H.dot(x)
    return z

#motion model Jacobian matrix   
def jacobiF(x, u):
    jF=np.array([
        [1.0, 0.0, -DT * speed * math.sin(steer), DT * math.cos(steer)],
        [0.0, 1.0, DT * speed * math.cos(steer), DT * math.sin(steer)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

#observation model Jacobian matrix
def jacobiH(x):
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]])
    return jH

#
def Kalman_filter(xEst, pEst, z, u):
    xPred = motion_model(xEst, u)
    jF = jacobiF(xPred, u)
    pPred = jF.dot(pEst).dot(jF.T) + R
    
    jH = jacobiH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH.dot(pPred).dot(jH.T) + Q
    K = pPred.dot(jH.T).dot(np.linalg.inv(S))
    xEst = xPred + K.dot(y)
    pEst = (np.eye(len(xEst)) - K.dot(jH)).dot(pPred)
    return xEst, pEst

def Kalman_start(begin):
    t0 = begin
    xEst = np.zeros((4,1))              # infinite a priori
    pEst = np.zeros((4,1))

# time - hms of sample
# speed - fps
# hdg - radians (x-axis zero)
# lonft - x-axis
# latft - y-axis
def Kalman_step(time, speed, hdg, lonft, latft):
    DT = time - t0
    t0 = time
    u = [speed, hdg]
    z = observation_model([latft, lonft])
    xEst, pEst = Kalman_filter(xEst, pEst, z, u)
    return xEst

test = [
    ["10:12:13", 60, 325, 7.123, 20.345],
    ["10:12:15", 60, 326, 7.146, 20.444],
    ]
for i in range(2):
    spd = test[i][1] * spdfactor
#    ang = np.deg2rad(450-test[0][2])
    ang = 3.14157/180*(450-test[i][2])
    x = test[i][3] + lonfeet
    y = test[i][4] * latfeet
    print (spd, ang, x, y)
    ]

    xnew = Kalman_step(spd, ang, x, y)

