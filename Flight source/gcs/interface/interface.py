#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import time

import numpy as np



class interface_run (Process):
    def __init__(self, status_in, data_out):
        """
        status_in and data_out must be queues.
        """
        Process.__init__(self)
        self.input_stream = status_in
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

        ####################
        # Operations frame
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
   
        ####################
        # Status frame
        statusFrame = LabelFrame(self.frame, text="Status", padx=5, pady=5)
        statusFrame.grid(row=0, column=1)
        
        # Set up binding variables - all strings
        self.tgtNum   = StringVar()
        self.tgtTheta = StringVar()
        self.tgtPhi   = StringVar()
        self.tgtR     = StringVar()
        
        Label(statusFrame, text="Target Number", anchor=E, justify=RIGHT).grid(row=0, column=0)
        Label(statusFrame, text="Target Theta",  anchor=E, justify=RIGHT).grid(row=1, column=0)
        Label(statusFrame, text="Target Phi",    anchor=E, justify=RIGHT).grid(row=2, column=0)
        Label(statusFrame, text="Target R",      anchor=E, justify=RIGHT).grid(row=3, column=0)

        Label(statusFrame, textvariable=self.tgtNum  , anchor=W, justify=LEFT).grid(row=0, column=1)
        Label(statusFrame, textvariable=self.tgtTheta, anchor=W, justify=LEFT).grid(row=1, column=1)
        Label(statusFrame, textvariable=self.tgtPhi  , anchor=W, justify=LEFT).grid(row=2, column=1)
        Label(statusFrame, textvariable=self.tgtR    , anchor=W, justify=LEFT).grid(row=3, column=1)
   
        self.tgtNum  .set("--")
        self.tgtTheta.set("--")
        self.tgtPhi  .set("--")
        self.tgtR    .set("--")
        

        root.mainloop()
