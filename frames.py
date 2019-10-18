#07_04_temp_final.py

from tkinter import *
import serial

ser = serial.Serial(port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

       
class App:
    
    def __init__(self, master):
#        self.t_conv = ScaleAndOffsetConverter('C', 'F', 1.8, 32)
        data = Frame(master)
        sta=Label(data,text="Status: ", font=(None,20))
        sta.grid(row=0,column=0)
        spd=Label(data,text="SPD:", font=(None,20))
        spd.grid(row=1,column=0)
        hdg=Label(data,text="HDG:", font=(None,20))
        hdg.grid(row=2,column=0)
        ste=Label(data,text="STR:", font=(None,20))
        ste.grid(row=3,column=0)
        dtg=Label(data,text="DTG:", font=(None,20))
        dtg.grid(row=4,column=0)
        ctg=Label(data,text="CTG:", font=(None,20))
        ctg.grid(row=5,column=0)
        xte=Label(data,text="XTE:", font=(None,20))
        xte.grid(row=6,column=0)
        lat=Label(data,text="Lat:", font=(None,20))
        lat.grid(row=7,column=0)
        lon=Label(data,text="Lon:", font=(None,16))
        lon.grid(row=8,column=0)

        self.status = StringVar()
        Label(data,width=8,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.status).grid(row=0,column=1)
        self.speed = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.speed).grid(row=1,column=1)
        self.head = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.head).grid(row=2,column=1)
        self.steer = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.steer).grid(row=3,column=1)
        self.dtg = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.dtg).grid(row=4,column=1)
        self.ctg = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.ctg).grid(row=5,column=1)
        self.xte = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.xte).grid(row=6,column=1)
        self.lat = StringVar()
        Label(data,width=12,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.lat).grid(row=7,column=1)
        self.lon= StringVar()
        Label(data, width=8,font=(None,16),bg="white",fg="blue",borderwidth=1,relief="solid",textvariable=self.lon).grid(row=8,column=1)

        
        keypad=Frame(master)
        b1=Button(keypad, text="1", command=self.one)
        b1.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b1.grid(row=0,column=0)
        b2=Button(keypad, text="2", command=self.two)
        b2.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b2.grid(row=0,column=1)
        b3=Button(keypad, text="3", command=self.three)
        b3.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b3.grid(row=0,column=2)
        bA=Button(keypad, text="A")
        bA.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bA.grid(row=0,column=3)
        b4=Button(keypad, text="4", command=self.four)
        b4.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b4.grid(row=1,column=0)
        b5=Button(keypad, text="5", command=self.five)
        b5.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b5.grid(row=1,column=1)
        b6=Button(keypad, text="6", command=self.six)
        b6.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b6.grid(row=1,column=2)
        bB=Button(keypad, text="B")
        bB.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bB.grid(row=1,column=3)
        b7=Button(keypad, text="7", command=self.seven)
        b7.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b7.grid(row=2,column=0)
        b8=Button(keypad, text="8", command=self.eight)
        b8.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b8.grid(row=2,column=1)
        b9=Button(keypad, text="9", command=self.nine)
        b9.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b9.grid(row=2,column=2)
        bC=Button(keypad, text="C")
        bC.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bC.grid(row=2,column=3)
        bS=Button(keypad, text="*", command=self.star_button)
        bS.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bS.grid(row=3,column=0)
        b0=Button(keypad, text="0")
        b0.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        b0.grid(row=3,column=1)
        bp=Button(keypad, text="#")
        bp.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bp.grid(row=3,column=2)
        bD=Button(keypad, text="D")
        bD.config(width=2,height=2,font=(None,25),bg="blue",fg="white")
        bD.grid(row=3,column=3)
        
        data.place(x=20,y=20)
#        data.grid(row=0,column=0)
#        keypad.grid(row=0,column=1)
        keypad.place(x=380,y=10)
        
        self.ibuffer = ""
        self.msg = ""
        self.exeflag = False
        self.lbflag = False
        self.lb2flag = False
        self.piflag = False
        
    def star_button(self):
        self.exeflag = True
        self.status.set("Auto")
        self.steer.set("-16")
        self.xte.set("3.45")
        self.lat.set("20.1255")
        self.lon.set("6.732")
    def pound(self):
        self.lbflag = True
        self.lb2flag = False
    def zero(self):
        self.key = '0'
        self.xmit()
    def one(self):
        self.key = '1'
        self.xmit()
    def two(self):
        self.key = '2'
        self.xmit()
    def three(self):
        self.key = '3'
        self.xmit()
    def four(self):
        self.key = '4'
        self.xmit()
    def five(self):
        self.key = '5'
        self.xmit()
    def six(self):
        self.key = '6'
        self.xmit()
    def seven(self):
        self.key = '7'
        self.xmit()
    def eight(self):
        self.key = '8'
        self.xmit()
    def nine(self):
        self.key = '9'
        self.xmit()
        
    def xmit(self):
        if (self.exeflag):
            self.msg = '{E' + self.key + '}'
            ser.write(self.msg.encode('utf-8'))
            self.exeflag = False
        elif (self.lbflag):
            if (self.lb2flag):
                self.msg = self.msg + self.key + '}'
                ser.write(msg.encode('utf-8'))
                self.lbflag = False
                self.lb2flag = False
            else:
                self.msg = '{F' + self.key
                self.lb2flag = True
        else:
            self.msg = '{D' + self.key + '}'
            ser.write(self.msg.encode('utf-8'))
        self.msg = ""

#   Listen to serial port for status info from rover pi
    def listen(self):
        while ser.in_waiting:
            inpt = ser.read(1).decode("utf-8")
            print(inpt)
            if (inpt == '{'):
                self.ihead = 0
                continue
            if (inpt == '}'):
                self.piflag = True
                break;
            self.ibuffer = self.ibuffer + inpt

           
        if self.piflag:
            xchar = self.ibuffer[0]
            lbuffer = self.ibuffer[1:]
            
            if (xchar == 'a'):
                self.status.set(lbuffer)
                    
            elif (xchar == 'c'):
                self.ctg.set(lbuffer)
                    
            elif (xchar == 'd'):
                self.dtg.set(lbuffer)
                    
            elif (xchar == 'h'):
                self.head.set(lbuffer)
                    
            elif (xchar == 'l'):
                self.lat.set(lbuffer)
                    
            elif (xchar == 'n'):
                self.lon.set(lbuffer)
                    
            elif (xchar == 's'):
                self.steer.set(lbuffer)
                    
            elif (xchar == 'v'):
                self.speed.set(lbuffer)
 
            self.piflag = False
                        
        root.after(25, self.listen)
  
        
root = Tk()
root.wm_title('Temp Converter')
app = App(root)
root.geometry("650x600+0+0")
root.after(25, app.listen)

root.mainloop()

