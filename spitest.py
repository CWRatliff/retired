import spidev
import time

buff="abcdef"
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

#    buff = spi.xfer2(msg)
    cbuff = chr(spi.xfer2(msg)[0])
    msg = [0x42]
#    buff += spi.xfer2(msg)
    cbuff += chr(spi.xfer2(msg)[0])
    msg = [0x43]
    cbuff += chr(spi.xfer2(msg)[0])
    msg = [0x44]
    cbuff += chr(spi.xfer2(msg)[0])
    msg = [0x45]
    cbuff += chr(spi.xfer2(msg)[0])

    msg=[0x0d]
    cbuff += chr(spi.xfer2(msg)[0])
    print(cbuff)
    time.sleep(5)
