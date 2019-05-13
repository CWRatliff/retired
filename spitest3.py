import spidev
import time

buff="{abcdef}"
cbuff = ""
i = 0;
NUL = [0x00]
spi = spidev.SpiDev()
rc = spi.open(0, 0)
spi.max_speed_hz = 125000
time.sleep(0.0005)

def spisend(cmd):
    i = 0;
    while True:
        c = cmd[i]
        spi.xfer(c)
        if (c == '}'):
            spi.xfer(0)
            break
        #endif
    #endwhile
        
while True:
    while True:
        c = spi.xfer(NUL)
        if (c == 0):
            break;
        if (c == '{'):
            cbuff = '{'
            continue
        cbuff += str([c])
        if (c == '}'):
            break;
    #endwhile
    print(cbuff)
    spisend(buff)
    time.sleep(5)

