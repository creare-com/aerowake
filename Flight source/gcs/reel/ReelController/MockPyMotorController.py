""" A mock interface to the Cython reelcontroller  """
import logging

class MockPyMotorController:
    def __init__(self):
        # Logger setup
        if 'reel_logger' in logging.Logger.manager.loggerDict:
            self._logger = logging.getLogger('reel_logger')
            print 'ReelController.py is using reel.py logger'
        else:
            self._logger = logging.getLogger('reel_logger')
            self._logger.setLevel(logging.DEBUG)
            print 'ReelController.py created its own reel_logger'
        self._opened = False
        self._enabled = False
        self._pos = 0;
        self._target_pos = 0;
        self._maxspeed = 0;
        self._maxad = 4294967295;
        self._profile = {'velocity':0, 'acceleration':0, 'deceleration':0}
        
    # Open/close the specified port. (throw an  exception on failure)
    def open(self):
        self._opened = True
    def close(self):
        self._opened = False
    def isOpen(self):
        return self._opened

    # Enable/disable movement. (throw an  exception on failure)
    def clearFaultAndEnable(self):
        self._enabled = True
        self._logger.info("Enabled mock motor controller")
    def clearFault(self):
        pass
    def disable(self):
        self._logger.info("Disabled mock motor controller")
        self._enabled = False
    def isEnabled(self):
        return self._enabled
    def isFaulted(self):
        return False
        
    # Configuration (throw an  exception on failure)
    def setSensorType(self, st):
        pass
    def setOperatingMode(self, st):
        pass
    def setEncoderSettings(self, pulses_per_turn=1024, invert_polarity=False):
        pass
    def getGearRatioNumerator(self):
        return 26 
    def getGearRatioDenominator(self):
        return 1
        
    # Movement (throw an  exception on failure)
    def moveToPosition(self, position):
        self._logger.info("Setting mock motor to %dQC"%position)
        self._target_pos = position
        self.setActualPosition(position)
    def getPosition(self):
        return self._pos
    def getTargetPosition(self):
        return self._target_pos
    def setActualPosition(self, actual_pos): # a method unique to the mock object
        self._pos = actual_pos
    def setMaxVelocity(self, velocity):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        self._maxspeed = velocity
        self._profile['velocity'] = min(velocity, self._profile['velocity'])
    def getMaxVelocity(self):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        return self._maxspeed
    def getMaxAccelDecel(self):
        """ in RPM/s after gearbox.  This is the limit that the controller will apply to the acceleration/deceleration of the position profile. """
        return self._maxad
    def setPositionProfile(self, velocity, acceleration, deceleration):
        """ velocity/accel/decel is in RPM or RPM/s after gearbox.  """
        self._profile = {'velocity':velocity, 'acceleration':acceleration, 'deceleration':deceleration}
    def getPositionProfile(self):
        """ (velocity, accel, decel) is in RPM or RPM/s after gearbox.  """
        return self._profile
    def haltMovement(self):
         pass
#        self._logger.info("Stopping movement in mock motor controller")

