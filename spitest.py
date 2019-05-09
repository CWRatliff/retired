import spidev
import time

buff="abcdef"
NUL = [0x00]
spi = spidev.SpiDev()
rc = spi.open(0, 0)
print ("rc = "+str(rc))
spi.max_speed_hz = 125000
time.sleep(0.0005)

while True:
    msg=[0x3f]     # '?' interrogatory
    spi.xfer2(msg)
    msg = [0x41]
    count = spi.xfer2(msg)
    print ("interrog count returned " + str(count[0]))
    cbuff = ""
    for i in range(0, count[0]):
        cbuff += chr(spi.xfer2(msg)[0])
    spi.xfer2(NUL)
    
    print(cbuff)
    time.sleep(5)
