#07_04_temp_final.py

from tkinter import *
#from converters import *

class App:
    
    def __init__(self, master):
#        self.t_conv = ScaleAndOffsetConverter('C', 'F', 1.8, 32)
        frame = Frame(master)
        frame.pack()
        Label(frame, text='Stat').grid(row=0, column=0)
        self.status = StringVar()
        Entry(frame, textvariable=self.status).grid(row=0, column=1)
        Label(frame, text='SPD').grid(row=1, column=0)
        self.result_var = StringVar()
        Label(frame, textvariable=self.result_var).grid(row=1, column=1)
        Label(frame, text='HDG').grid(row=2, column=0)
        Label(frame, text='AZ').grid(row=3, column=0)
        Label(frame, text='DTW').grid(row=4, column=0)
        Label(frame, text='Lat').grid(row=5, column=0)
        Label(frame, text='Lon').grid(row=6, column=0)
        button = Button(frame, text='1', command=self.convert)
        button.grid(row=0, column=2, columnspan=1)
        button = Button(frame, text='2', command=self.convert)
        button.grid(row=0, column=3, columnspan=1)
        button = Button(frame, text='3', command=self.convert)
        button.grid(row=0, column=4, columnspan=1)
        button = Button(frame, text='A', command=self.convert)
        button.grid(row=0, column=5, columnspan=1)
        button = Button(frame, text='4', command=self.convert)
        button.grid(row=1, column=2, columnspan=1)
        button = Button(frame, text='5', command=self.convert)
        button.grid(row=1, column=3, columnspan=1)
        button = Button(frame, text='6', command=self.convert)
        button.grid(row=1, column=4, columnspan=1)
        button = Button(frame, text='B', command=self.convert)
        button.grid(row=1, column=5, columnspan=1)
        button = Button(frame, text='7', command=self.convert)
        button.grid(row=2, column=2, columnspan=1)
        button = Button(frame, text='8', command=self.convert)
        button.grid(row=2, column=3, columnspan=1)
        button = Button(frame, text='9', command=self.convert)
        button.grid(row=2, column=4, columnspan=1)
        button = Button(frame, text='C', command=self.convert)
        button.grid(row=2, column=5, columnspan=1)
        button = Button(frame, text='*', command=self.star_button)
        button.grid(row=3, column=2, columnspan=1)
        button = Button(frame, text='0', command=self.convert)
        button.grid(row=3, column=3, columnspan=1)
        button = Button(frame, text='#', command=self.convert)
        button.grid(row=3, column=4, columnspan=1)
        button = Button(frame, text='D', command=self.convert)
        button.grid(row=3, column=5, columnspan=1)

    def convert(self):
        c = self.c_var.get()
        self.result_var.set(self.t_conv.convert(c))
        
    def star_button(self):
        self.status.set("Auto")

root = Tk()
root.wm_title('Temp Converter')
app = App(root)
root.mainloop()
