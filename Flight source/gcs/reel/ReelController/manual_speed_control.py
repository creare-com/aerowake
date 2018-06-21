"""
Retracts the tether at a fixed rate until interrupted

"""
import time
import math
import logging
from PyMotorController import *

def open():
    global mc
    mc = PyMotorController()
    mc.setOperatingMode('PROFILE_VELOCITY')
    max_ad = mc.getMaxAccelDecel()
    mc.setVelocityProfile(max_ad, max_ad) # no smoothing
    mc.clearFaultAndEnable() # movement may occur after this point

speed_rpm = 0
speed_inc = 5
def updateSpeed():
    global mc
    mc.moveWithVelocity(speed_rpm)
    
def holdPosition():
    global mc
    mc.haltMovement()
    
if __name__ == "__main__":
    global mc
    try:
        stationary = False
        open()
        updateSpeed()
        while True:
            print("Press ctrl-C at any time to exit.  This will put the reel in a freewheeling state.")
            cmd=raw_input('Type\n  h<enter> to pause and hold position,\n  r<enter> to resume,\n  f<enter> to go faster (reel more inwards),\n  s<enter> to go slower (let out more line).\n')
            if   cmd.startswith('h'):
                stationary = True
                holdPosition()
                print("Holding position.")
            elif cmd.startswith('r'):
                stationary = False
                updateSpeed()
                print("Resumed movement.")
            elif cmd.startswith('f'):
                speed_rpm += speed_inc
                if not stationary:
                    updateSpeed()
                print("Speed is now %dRPM."%speed_rpm)
                if stationary:
                    print("Type r<enter> to resume movement.")
            elif cmd.startswith('s'):
                speed_rpm -= speed_inc
                if not stationary:
                    updateSpeed()
                print("Speed is now %dRPM."%speed_rpm)
                if stationary:
                    print("Type r<enter> to resume movement.")
    except KeyboardInterrupt:
        print("Caught ^C; exiting.")
    finally:
        print("Disabling motor control; reel will now spin freely.")
        del mc
