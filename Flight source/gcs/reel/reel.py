#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import logging
import sys
import time
import ReelController
import logging

SKIP_CYCLES=100 #only send the position every 10 cycles
# SKIP_CYCLES=0

class reel_run (Process):
    def __init__(self, cmd, data_out):
        Process.__init__(self)
        self._cmd = cmd
        self._data_out = data_out
        self._dt_des =1/200.0
        self._cycles = 0

        # Setup logger
        # NOTE: Logger is singleton as long as handled by the same Python interpreter. Calling logging.getLogger('logger_name') from multiple scripts on the same computer will use the same file (except in advanced cases not relevant here).
        logger = logging.getLogger('gcs_logger')

    def run(self):
        run = True
        self._rc = ReelController.ReelController()
        while run:
            #check if any new cmds
            try:
                cmd = self._cmd.get(False)
            except Empty:
                cmd = None

            # We expect cmd to be one of the following:
            # {"cmd":"goto", "L":<length in meters, an int}
            # {"cmd":"halt"}    # holds the reel where it is
            # {"cmd":"rehome"}  # consider the current tether length to be 0m
            # {"cmd":"exit"}    # exit the subprocess
            if cmd != None and cmd.has_key('cmd'):
                if cmd['cmd'] == 'goto':
                    L = cmd['L']
                    logging.info("Setting tether length to %fm"%L)
                    self._rc.setTetherLengthM(L)
                elif cmd['cmd'] == 'rehome':
                    logging.info("Homing tether")
                    self._rc.youAreHome()
                elif cmd['cmd'] == 'halt':
                    logging.info("Halting reel")
                    self._rc.stopMoving()
                elif cmd['cmd'] == 'exit':
                    logging.info("Halting reel")
                    self._rc.stopMoving()
                    del self._rc
                    run = False
                    continue
            
            t_0 = datetime.datetime.now()

            # Push tether status back if we've skipped the appropriate number of cycles
            self._rc.update()
            if(self._cycles >= SKIP_CYCLES):
                L = self._rc.getTetherLengthM()
                T = self._rc.getTetherTensionN()
                try:
                    self._data_out.get(False)
                except Empty:
                    # GCS main code will occasionally pull out this data, so we need to protect against an empty Queue
                    pass
                self._data_out.put({"L": L, "T": T})
                self._cycles = 0
            else:
                self._cycles += 1

            #Sleep for desired amount of time
            t_1 = datetime.datetime.now()
            dt = (t_1-t_0).total_seconds()
            sleep_time = self._dt_des-dt
            if sleep_time<0:
                sleep_time = 0
            time.sleep(sleep_time)


