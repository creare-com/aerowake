# Run as ipython -i test_script.py
import time
import logging
logging.basicConfig(level=logging.INFO)
import ReelController
rc = ReelController.ReelController()

dt_des = 1/200.0

def setAndLoop(len_m):
    rc.setTetherLengthM(len_m)
    t_0 = time.time()
    while True:

        rc.update()

        # Cap frame rate (at something fairly high)
        t_1 = time.time()
        dt = (t_1-t_0)
        sleep_time = dt_des-dt
        if sleep_time<0:
            sleep_time = 0
        time.sleep(sleep_time)
        t_0 = t_1
