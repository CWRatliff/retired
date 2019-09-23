import cEKF
import numpy as np
import math

spdfactor = .0035
d3 = 10.5
d1 = 7.254
lonfeet = 83.716
latfeet = 101.267
ang = 0
test = [
    [34, 60, -6, 7.082, 20.532],
    [37, 60, -10, 7.082, 20.545],
    [43, 60, 1, 7.082, 20.559],
    [45, 60, 4, 7.138, 20.559],
    ]

Kfilter = cEKF.Kalman_filter()

Kfilter.Kalman_start(32, -7.082 * lonfeet, 20.532 * latfeet, -2.96, 60*spdfactor)
for i in range(4):
    time = test[i][0]

    v = test[i][1] * spdfactor
    steer = test[i][2]
    alpha = np.deg2rad(steer)
    h = d3/math.sin(alpha)
    turnrad = (h * math.cos(alpha) + d1) / 12
    omega = v / turnrad
    xl = -test[i][3] * lonfeet
    yl = test[i][4] * latfeet

    print (v, omega, xl, yl)
    xest = Kfilter.Kalman_step(time, xl, yl, omega, v)
    print("Kalman new est", xest)