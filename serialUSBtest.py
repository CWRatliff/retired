import serial
port = "/dev/ttyUSB0"
s1 = serial.Serial(port, 9600)
s1.flushInput()
msgStart = False
msgEnd = False
test = "{test}"

while True:
    while True:
        if s1.inWaiting() > 0:
            try:
                inp = s1.read(1).decode("utf-8")
            except UnicodeDecodeError:
                msgStart = False
                print("Woops")
                continue
            if (inp == '{'):
                msgStart = True
                msg = inp
                continue
            if msgStart:
                msg += inp
                if (inp == '}'):
                    print(msg)
                    msgEnd = True
                    msgStart = False
                break
            
    if (msgEnd):
        s1.write(test.encode("utf-8"))
        msgEnd = False
            
