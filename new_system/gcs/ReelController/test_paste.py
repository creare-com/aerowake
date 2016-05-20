# Meant to be pasted into an ipython console
import time
import logging
logging.basicConfig(level=logging.INFO)
import ReelController
rc = ReelController.ReelController()
rc.getTetherLengthM()
rc._home_pos_m
def setAndLoop(len_m):
rc.setTetherLengthM(len_m)
while True:
rc.update()
time.sleep(0.05)

