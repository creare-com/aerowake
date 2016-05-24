"""
Retracts the tether at a fixed rate until interrupted

"""
import time
import math
import logging
from PyMotorController import *

def open():
    mc = PyMotorController()
    mc.setOperatingMode('PROFILE_VELOCITY')
    mc.clearFaultAndEnable() # movement may occur after this point

speed_rpm = 20
speed_inc = 5
def updateSpeed():
    mc.moveWithVelocity(-speed_rpm)
    
def holdPosition():
    mc.haltMovement()
    
if __name__ == "__main__":
    try:
        stationary = False
        open()
        updateSpeed()
        while True:
            print("Press ctrl-C at any time to exit.  This will put the reel in a freewheeling state.")
            cmd=raw_input('Type h<enter> to pause and hold position,\n  r<enter> to resume,\n f<enter> to go faster,\n  s<enter> to go slower.')
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
        mc.disable()
        mc.close()