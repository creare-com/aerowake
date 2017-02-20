import datetime
import time

class reel_run:
    def __init__(self,dt):
        

        # USER CONFIGURABLE SETTINGS
        self.dt_des = dt
        time.sleep(.1)


    ## Functions
    

################################################################################
################################################################################

#This will get called at 200hz or whatever rate you want it to, as specified by the airprobe_run().

    def run(self):
        reel_data = [0,0,0,0,0]        
        #log data

        return reel_data

