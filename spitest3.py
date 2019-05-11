import spidev
import time

buff="abcdef"
cbuff = ""
i = 0;
spi = spidev.SpiDev()
rc = spi.open(0, 0)
spi.max_speed_hz = 125000
time.sleep(0.0005)

while True:
    while True:
        msg = spi.xfer(buff[i])
        cbuff += msg[0]
        i += 1;
        if (msg[0] == 0):
            break;
    print(cbuff)
    cbuff= ""
    i = 0
    time.sleep(5)

