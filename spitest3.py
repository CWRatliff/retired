import spidev
import time

buff="{abcdef}"
cbuff = ""
flag = False
i = 0;
NUL = [0x00]
spi = spidev.SpiDev()
rc = spi.open(0, 0)
spi.max_speed_hz = 125000
time.sleep(0.0005)
c = spi.xfer(NUL)           #flush stale data
epoch = time.time()

# send a command string to slave
def spisend(cmd):
    i = 0;
    while True:
        d = cmd[i]
        c = [ord(d)]
        spi.xfer(c)
        if (d == '}'):      # end of string
            spi.xfer(NUL)   # flush last char
            break
            #endif
        i += 1
        #endwhile
        
while True:
    # read as many chars from slave as are available
    while True:
        c = spi.xfer(NUL)
        if (c[0] == 0):
            break;
        d = chr(c[0])
        if (d == '{'):
            cbuff = '{'
            continue
        cbuff += d
        if (d == '}'):
            flag = True
            break;
        #endwhile

    if (flag):   
        print(cbuff)
        flag = False
    spisend(buff)
    time.sleep(5)

