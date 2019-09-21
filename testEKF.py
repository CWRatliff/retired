import cEKF
import numpy as np

spdfactor = .005
lonfeet = 1 #82.0
latfeet = 1 #100.0
t0 = 0
ang = 0

Kfilter = cEKF.Kalman_filter()
test = [
    [1, 60, 10, .1, .2],
    [2, 60, 15, .2, .4],
    ]
for i in range(2):
    deltaT = test[i][0] - t0
    t0 = test[i][0]
    omega = (test[i][2] - ang) / deltaT
    ang = test[i][2]
    spd = test[i][1] * spdfactor
    omega = np.deg2rad(omega)
    x = -test[i][3] + lonfeet
    y = test[i][4] * latfeet

    print (spd, ang, x, y)
    xest = Kfilter.Kalman_step(deltaT, spd, omega, x, y)
    print("Kalman new est", xest)