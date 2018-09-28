#!/usr/bin/env python2

import datetime
from multiprocessing import Process, Queue
from Queue import Empty
import logging
import os
import sys
import time
import ReelController

SKIP_CYCLES=100 #only send the position every 10 cycles
# SKIP_CYCLES=0

class reel_run (Process):
    def __init__(self, cmd, data_out):
        Process.__init__(self)
        self._cmd = cmd
        self._data_out = data_out
        self._dt_des =1/200.0
        self._cycles = 0

        # Logger setup
        self._logger = logging.getLogger('reel_logger')
        self._logger.setLevel(logging.DEBUG)
        # Create file handler that sends all logger messages (DEBUG and above) to file
        logfile = '%s/logs/reel-logs/reel-%s.log' %(os.path.expanduser('~'),time.strftime('%Y-%m-%d-%Hh-%Mm-%Ss', time.localtime()))
        fh = logging.FileHandler(logfile)
        print 'Logging reel data to %s' %(logfile)
        fh.setLevel(logging.DEBUG)
        # Create console handler that sends some messages (INFO and above) to screen
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(logging.INFO)
        # Set the log format for each handler
        form_fh = logging.Formatter('%(created)s,%(relativeCreated)s,%(funcName)s,%(levelname)s: %(message)s')
        form_ch = logging.Formatter('%(levelname)s: %(message)s')
        fh.setFormatter(form_fh)
        ch.setFormatter(form_ch)
        # Add the handler to the logger
        self._logger.addHandler(fh)
        self._logger.addHandler(ch)

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
                    self._logger.info("Setting tether length to %0.01f m"%L)
                    self._rc.setTetherLengthM(L)
                elif cmd['cmd'] == 'rehome':
                    self._logger.info("Homing tether\n")
                    self._rc.youAreHome()
                elif cmd['cmd'] == 'halt':
                    self._logger.info("Halting reel\n")
                    self._rc.stopMoving()
                elif cmd['cmd'] == 'exit':
                    self._logger.info("Halting reel\n")
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
                # Log length and tension reading to GCS log
                self._logger.debug('tetherDATA,(L,T) = (%.01f,%.01f)',L,T)
            else:
                self._cycles += 1

            #Sleep for desired amount of time
            t_1 = datetime.datetime.now()
            dt = (t_1-t_0).total_seconds()
            sleep_time = self._dt_des-dt
            if sleep_time<0:
                sleep_time = 0
            time.sleep(sleep_time)


