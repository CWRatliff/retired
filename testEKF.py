import dEKF
#import numpy as np
import math
import time

print("time", time.time())
spdfactor = .008
d3 = 10.5
d1 = 7.254
lonfeet = -83.716
latfeet = 101.267
ang = 0
test = [
    [15, 50, 336, 7.0798, 22.4219],
    [17, 50, 336, 7.0808, 22.4219],
    [18, 50, 335, 7.0837, 22.428],
    [19, 50, 336, 7.0859, 22.4312],
    ]

Kfilter = dEKF.Kalman_filter()

Kfilter.Kalman_start(14, 7.0798 * lonfeet, 22.4190 * latfeet, math.radians(450-336), 50*spdfactor)
for i in range(4):
    time = test[i][0]

    v = test[i][1] * spdfactor
    hdg = test[i][2]

    xl = test[i][3] * lonfeet
    yl = test[i][4] * latfeet
    phi = math.radians(450-hdg)

    print (v, phi, xl, yl)
    xest = Kfilter.Kalman_step(time, xl, yl, phi, v)
    print("Kalman new est", xest[0]/lonfeet, xest[1]/latfeet, 450-math.degrees(xest[2]), xest[3])