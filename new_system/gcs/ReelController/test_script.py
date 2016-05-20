# Run as ipython -i test_script.py
import time
import logging
logging.basicConfig(level=logging.INFO)
import ReelController
rc = ReelController.ReelController()
def setAndLoop(len_m):
    rc.setTetherLengthM(len_m)
    while True:
        rc.update()
        time.sleep(0.05)

