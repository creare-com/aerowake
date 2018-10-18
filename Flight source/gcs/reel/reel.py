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
    def __init__(self, cmd, data_out, filename):
        Process.__init__(self)
        self._cmd = cmd
        self._data_out = data_out
        self._dt_des =1/200.0
        self._cycles = 0
        self._curr_len = 0

        # Logger setup
        self._logger = logging.getLogger('reel_logger')
        self._logger.setLevel(logging.DEBUG)
        # Create file handler that sends all logger messages (DEBUG and above) to file
        logfile = '%s/logs/reel-logs/%s-reel-%s.log' %(os.path.expanduser('~'),filename,time.strftime('%Y-%m-%d-%Hh-%Mm-%Ss', time.localtime()))
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
        try:
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
                # {"cmd":"clearfault"}  # clear any fault
                # {"cmd":"clearfaultandenable"}  # clear any fault and re-enable controller
                # {"cmd":"exit"}    # exit the subprocess and freewheel motor
                if cmd != None and cmd.has_key('cmd'):
                    if cmd['cmd'] == 'goto':
                        L = cmd['L']
                        self._logger.info("reel.py: Setting tether length to %0.01f m"%L)
                        self._rc.setTetherLengthM(L)
                    elif cmd['cmd'] == 'rehome':
                        self._logger.info("reel.py: Homing tether\n")
                        self._rc.youAreHome()
                    elif cmd['cmd'] == 'halt':
                        self._logger.info("reel.py: Halting reel via halt\n")
                        self._rc.stopMoving()
                    elif cmd['cmd'] == 'clearfault':
                        self._logger.info("reel.py: Clearing fault\n")
                        self._rc.clearFault()
                    elif cmd['cmd'] == 'clearfaultandenable':
                        self._logger.info("reel.py: Clearing fault and enabling\n")
                        self._rc.clearFaultAndEnable()
			self._rc.setTetherLengthM(self._rc.getTetherLengthM())
		    elif cmd['cmd'] == 'geterror':
                        self._logger.info("reel.py: Printing device errors\n")
			self._logger.info(self._rc._mc.getDeviceErrors())	
                    elif cmd['cmd'] == 'exit':
                        self._logger.info("reel.py: Halting reel via exit\n")
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
                    faulted = self._rc.isFaulted()
                    enabled = self._rc.isEnabled()
                    try:
                        self._data_out.get(False)
                    except Empty:
                        # GCS main code will occasionally pull out this data, so we need to protect against an empty Queue
                        pass
                    self._data_out.put({"L": L, "T": T, "Faulted": faulted, "Enabled":enabled})
                    self._cycles = 0
                    self._curr_len = L
                else:
                    self._cycles += 1

                #Sleep for desired amount of time
                t_1 = datetime.datetime.now()
                dt = (t_1-t_0).total_seconds()
                sleep_time = self._dt_des-dt
                if sleep_time<0:
                    sleep_time = 0
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            self._logger.info('Reel got CTRL+C. Stopping reel.')
            self._rc.stopMoving()
            del self._rc



