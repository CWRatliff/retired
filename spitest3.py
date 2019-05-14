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
epoch = time.time()

def spisend(cmd):
    i = 0;
    while True:
        d = cmd[i]
        c = [ord(d)]
        spi.xfer(c)
        if (d == '}'):
            spi.xfer(NUL)
            break
            #endif
        i += 1
        #endwhile
        
while True:
    while True:
        c = spi.xfer(NUL)
        if (c == 0):
            break;
        d = chr(c[0])
        if (d == '{'):
            cbuff = '{'
            continue
        cbuff += d
        if (d == '}'):
            break;
        #endwhile
    print(cbuff)
    spisend(buff)
    time.sleep(5)

