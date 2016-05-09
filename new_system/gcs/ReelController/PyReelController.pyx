""" Adapter classes used to interface with the C++ ReelController class """

from libcpp.string cimport string
from libcpp cimport bool

cdef extern from "ReelController.hpp" namespace "gcs":
    cdef cppclass ReelController:
        ReelController(string, double) except +
        void haltMovement() except + # Cancel last movement - tries to hold the tether at this length
        void disable() except + #  permit the tether to spool freely
        void clearFaultAndEnable() except + #  enable movement control; hold tether at this length
        bool isEnabled() except + # returns true if the motor controller is attempting to hold position
        void setTetherToHome() except + # consider the tether length to be 0, we're fully reeled in.
        void setTetherLength(double) except + # pays out or reels in the tether to this length
        double setMaxTetherSpeed(double) except + # Returns the actual payout rate set.  Will cap based on the motor & gearbox capabilities.
        double getMaxTetherSpeed() except + # Returns the actual payout/retract rate.
        double getTetherLength() except +
        double getTetherTargetLength() except +

cdef class PyReelController:
    cdef ReelController *rc
    def __cinit__(self, string usb_port = "USB0", double reel_diam_cm = 12.7):
        self.rc = new ReelController(usb_port, reel_diam_cm)
    def __dealloc__(self):
        del self.rc
        
    def haltMovement(self): # Cancel last movement - tries to hold the tether at this length
        return self.rc.haltMovement()
    def disable(self): #  permit the tether to spool freely
        return self.rc.disable()
    def clearFaultAndEnable(self): #  enable movement control; hold tether at this length
        return self.rc.clearFaultAndEnable()
    def isEnabled(self): # returns true if the motor controller is attempting to hold position
        return True if self.rc.isEnabled() else False
        
    # def setTetherToHome(self): # consider the tether length to be 0, we're fully reeled in.
        # raise NotImplementedException()
        # return self.rc.setTetherToHome()
    def setTetherLength(self, double desired_length_m): # pays out or reels in the tether to this length
        """ Commands the current length of the tether, in meters. """
        return self.rc.setTetherLength(desired_length_m)
    def setMaxTetherSpeed(self, double max_tether_mps): # Returns the actual payout rate set.  Will cap based on the motor & gearbox capabilities.
        return self.rc.setMaxTetherSpeed(max_tether_mps)
    def getMaxTetherSpeed(self): # Returns the actual payout rate set.
        return self.rc.getMaxTetherSpeed()
    def getTetherLength(self):
        """ Returns the current length of the tether, in meters. """
        return self.rc.getTetherLength()
    def getTetherTargetLength(self):
        """ Returns the length to which the reel is spooling, in meters. """
        return self.rc.getTetherLength()
