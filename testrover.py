import sys
import serial
import time


epoch = time.time()
cbuff = ""

flag = False

try:

        while True:                         # read characters from slave
            if (tty.inWaiting() == 0):
                break

            try:
                d = tty.read(1).decode("utf-8")
                print(d)
            except UnicodeDecodeError:
                cbuff = ""
                msgStart = False
                print("Woops")
                continue
            
            if (d == '{'):
                cbuff = "{"
                msgStart = True
                flag = False
                print("start")
                break
            if msgStart:
                cbuff += d
                print(cbuff)
                if (d == '}'):
                    flag = True
                    msgStart = False
                    tt=time.localtime()
                    ts=time.strftime("%H:%M:%S ", tt)
                    print("msg: " + ts + cbuff)
                break
            #endwhile read
        pass
    pass
