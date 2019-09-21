import cEKF
import numpy as np

spdfactor = .005
lonfeet = 1 #82.0
latfeet = 1 #100.0
t0 = 0
ang = 0
test = [
    [1, 60, 10, .1, .2],
    [2, 60, 15, .2, .4],
    ]

Kfilter = cEKF.Kalman_filter()

Kfilter.Kalman_start(0, 0, 0, 0, 60*spdfactor)
for i in range(2):
    deltaT = test[i][0] - t0
    t0 = test[i][0]
    omega = (test[i][2] - ang) / deltaT
    ang = test[i][2]
    spd = test[i][1] * spdfactor
    omega = np.deg2rad(omega)
    xl = -test[i][3] * lonfeet
    yl = test[i][4] * latfeet

    print (spd, omega, xl, yl)
    xest = Kfilter.Kalman_step(deltaT, spd, omega, xl, yl)
    print("Kalman new est", xest)