#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import time

import numpy as np



class interface_run (Process):
    def __init__(self, cmd, data_out):
        Process.__init__(self)
        self.input_stream = cmd
        self.curr_state = 999
        self.data_out = data_out


    def callback_takeoff(self):
        print "Takeoff Vehicle"
        self.data_out.put("TAKEOFF_CMD")

    def callback_land(self):
        print "Land Vehicle"
        self.data_out.put("LAND_CMD")

    def callback_auto(self):
        print "Auto Mode"
        self.data_out.put("AUTO_CMD")

    def callback_adv(self):
        print "Advance Target"
        self.data_out.put("ADV_CMD")
    
    def callback_quit(self):
        print "Quit"
        self.data_out.put("QUIT_CMD")
        self.frame.quit()

    def callback_halt_reel(self):
        print "Halt reel"
        self.data_out.put("HALT_REEL_CMD")

    def update_curr(self):
        # print "updating"
        # self.l1.config(text = "done")
        try:
            self.curr_state = self.input_stream.get(False)
        except Empty:
            pass

    
    def run(self):
        from Tkinter import * # To address the issue described in http://stackoverflow.com/questions/10755641/

        root = Tk()
        self.frame = Frame(root)
        self.frame.pack()

        commandsFrame = LabelFrame(self.frame, text="Operations")
        commandsFrame.grid(row=0, column=0)


        b1 = Button(commandsFrame, text="QUIT", fg="red",command=self.callback_quit)
        b1.pack(fill=X)

        self.buttonTakeoff = Button(commandsFrame, text="Takeoff", command=self.callback_takeoff)
        self.buttonTakeoff.pack(fill=X)

        self.buttonLand = Button(commandsFrame, text="Land", command=self.callback_land)
        self.buttonLand.pack(fill=X)

        self.buttonAutoMode = Button(commandsFrame, text="Auto Mode",command=self.callback_auto)
        self.buttonAutoMode.pack(fill=X)

        self.buttonHaltReel = Button(commandsFrame, text="Halt reel",command=self.callback_halt_reel)
        self.buttonHaltReel.pack(fill=X)

        # self.l1 = Label(root, fg="dark green")
        # self.l1.pack(fill=X)

        # curr = str(self.curr_state)
        # self.l1.config(text="eyyyyyy")
        # self.l1.after(1000,self.update_curr)

        self.buttonAdvTgt = Button(commandsFrame, text="Advance Target", command=self.callback_adv)
        self.buttonAdvTgt.pack(fill=X)
   
        root.mainloop()
