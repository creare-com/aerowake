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
        status_in is expected to be populated with lists of pairs like [(name, value)],
            where name is a string and value is anything that can be passed to str().
        """
        Process.__init__(self)
        self.input_stream = status_in
        self.data_out = data_out
        self._UPDATE_INTERVAL_MS = 10
        self._status_labels = {} # A dictionary, whose keys are strings and whose values are Tkinter label variables


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
    
    def callback_resend(self):
        print "Resend a target"
        self.data_out.put("RESEND_CMD")
    
    def callback_dec(self):
        print "Go back a target"
        self.data_out.put("DEC_CMD")
    
    def callback_quit(self):
        print "Quit"
        self.data_out.put("QUIT_CMD")
        self.frame.quit()

    def callback_halt_reel(self):
        print "Halt reel"
        self.data_out.put("HALT_REEL_CMD")

    def update_status(self):
        from Tkinter import * # To address the issue described in http://stackoverflow.com/questions/10755641/
        try:
            state_vars = self.input_stream.get(False)
        except Empty:
            state_vars = []
        
        for name,val in state_vars:
            if not name in self._status_labels:
                # Make a new label
                row_num = len(self._status_labels)
                self._status_labels[name] = StringVar()
                Label(self.statusFrame, text=name+":", justify=RIGHT).grid(row=row_num, column=0, sticky=N+S+E+W)
                Label(self.statusFrame, textvariable=self._status_labels[name], justify=LEFT).grid(row=row_num, column=1, sticky=N+S+E+W)
            self._status_labels[name].set(str(val))
        
        # Again
        self.frame.after(self._UPDATE_INTERVAL_MS,self.update_status)

    
    def run(self):
        from Tkinter import * # To address the issue described in http://stackoverflow.com/questions/10755641/

        root = Tk()
        
        root.protocol("WM_DELETE_WINDOW", self.callback_quit)
        self.frame = Frame(root)
        self.frame.pack()
        self.frame.after(self._UPDATE_INTERVAL_MS,self.update_status)

        ####################
        # Operations frame
        commandsFrame = LabelFrame(self.frame, text="Operations")
        commandsFrame.grid(row=0, column=0)

        self.buttonTakeoff = Button(commandsFrame, text="Takeoff", command=self.callback_takeoff)
        self.buttonTakeoff.pack(fill=X)

        self.buttonLand = Button(commandsFrame, text="Land", command=self.callback_land)
        self.buttonLand.pack(fill=X)

        self.buttonAutoMode = Button(commandsFrame, text="Enable positional seeking",command=self.callback_auto)
        self.buttonAutoMode.pack(fill=X)

        self.buttonHaltReel = Button(commandsFrame, text="Halt reel",command=self.callback_halt_reel)
        self.buttonHaltReel.pack(fill=X)

        self.buttonAdvTgt = Button(commandsFrame, text="Advance Target >>", command=self.callback_adv)
        self.buttonAdvTgt.pack(fill=X)
   
        self.buttonAdvTgt = Button(commandsFrame, text="Resend Target", command=self.callback_resend)
        self.buttonAdvTgt.pack(fill=X)
   
        self.buttonAdvTgt = Button(commandsFrame, text="<< Decrement Target", command=self.callback_dec)
        self.buttonAdvTgt.pack(fill=X)
   
        ####################
        # Status frame
        self.statusFrame = LabelFrame(self.frame, text="Status", padx=5, pady=5)
        self.statusFrame.grid(row=0, column=1)
        # Grid.columnconfigure(self.statusFrame, 0, weight=1)
        # Grid.columnconfigure(self.statusFrame, 1, weight=1)

        root.mainloop()
