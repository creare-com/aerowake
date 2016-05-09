""" A mock interface to the Cython reelcontroller  """
import logging

class MockPyReelController:
    def __init__(self):
        self._enabled = False
        self._len = 0;
        self._target_len = 0;
        self._maxspeed = 0;
    def haltMovement(self): # Cancel last movement - tries to hold the tether at this length
        logging.info("Mock reel controller: halting movement")
        pass
    def disable(self): #  permit the tether to spool freely
        logging.info("Mock reel controller: disabling")
        self._enabled = False
    def clearFaultAndEnable(self): #  enable movement control; hold tether at this length
        logging.info("Mock reel controller: enabling")
        self._enabled = True
    def isEnabled(self): # returns true if the motor controller is attempting to hold position
        return self._enabled
        
    def setTetherLength(self, desired_length_m): # pays out or reels in the tether to this length
        logging.info("Mock reel controller: setting desired length to %fm."%desired_length_m)
        self._target_len = desired_length_m
    def setMaxTetherSpeed(self, max_tether_mps): # Returns the actual payout rate set.  Will cap based on the motor & gearbox capabilities.
        logging.info("Mock reel controller: setting max speed to %fm/s."%max_tether_mps)
        self._maxspeed = max_tether_mps
    def getMaxTetherSpeed(self): # Returns the actual payout rate set.
        return self._maxspeed
    def getTetherLength(self):
        return self._len
    def getTetherTargetLength(self):
        return self._target_len

    def setActualLength(self, len):
        """ Only for use in the mock system """ 
        self._len = len
