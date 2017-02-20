#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import time

import numpy as np
from reel_main import reel_main

SKIP_CYCLES=10 #only send the position every 10 cycles

class reel_controller (Process):
    def __init__(self, cmd, data_out):
        Process.__init__(self)
        self.cmd = cmd
        self.data_out = data_out
        self.dt_des =1/200.0
        self.reel = reel_main(self.dt_des)
        self.cycles = 0

    def run(self):

        while True:
            #check if any new cmds
            try:
                self.cmd.get(False)
            except Empty:
                pass

            t_0 = datetime.datetime.now()

            #run the airprobe and push the data to the queue if its been so many skip cycles. 
            if(self.cycles >= SKIP_CYCLES):
                self.data_out.put(self.reel.run())
                self.cycles = 0
            else:
                self.reel.run()
                self.cycles = self.cycles + 1

            #Sleep for desired amount of time
            t_1 = datetime.datetime.now()
            dt = (t_1-t_0).total_seconds()
            sleep_time = self.dt_des-dt
            if sleep_time<0:
                sleep_time = 0
            time.sleep(sleep_time)


