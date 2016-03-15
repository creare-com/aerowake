#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import time

import numpy as np

from Tkinter import *


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

    def update_curr(self):
        print "updating"
        try:
            self.curr_state = self.input_stream.get(False)
        except Empty:
            pass

    def run(self):

        root = Tk()
        frame = Frame(root)
        frame.pack()



        b1 = Button(frame, text="QUIT", fg="red",command=frame.quit)
        b1.pack(fill=X)

        b2 = Button(frame, text="Takeoff", command=self.callback_takeoff)
        b2.pack(fill=X)

        b3 = Button(frame, text="Land", command=self.callback_land)
        b3.pack(fill=X)

        b6 = Button(frame, text="Auto Mode",command=self.callback_auto)
        b6.pack(fill=X)

        l1 = Label(root, fg="dark green")
        l1.pack(fill=X)

        curr = str(self.curr_state)
        l1.config(text=curr)
        l1.after(1,self.update_curr)

        b4 = Button(frame, text="Advance Target", command=self.callback_adv)
        b4.pack(fill=X)


        b5 = Button(frame, text="Update", command=self.update_curr)
        b5.pack(fill=X)

        root.mainloop()

        

        # while True:
        #     #check if any new cmds
        #     try:
        #         self.cmd.get(False)
        #     except Empty:
        #         pass

        #     t_0 = datetime.datetime.now()

        #     #run the airprobe and push the data to the queue if its been so many skip cycles. 
        #     if(self.cycles >= SKIP_CYCLES):
        #         self.data_out.put(self.airprobe.run())
        #         self.cycles = 0
        #     else:
        #         self.airprobe.run()
        #         self.cycles = self.cycles + 1

        #     #Sleep for desired amount of time
        #     t_1 = datetime.datetime.now()
        #     dt = (t_1-t_0).total_seconds()
        #     sleep_time = self.dt_des-dt
        #     if sleep_time<0:
        #         sleep_time = 0
        #     time.sleep(sleep_time)

