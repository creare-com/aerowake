from libcpp.string cimport string

cdef extern from "ReelController.hpp" namespace "gcs":
    cdef cppclass ReelController:
        ReelController(string, double) except +
        void setTetherToHome() except + # consider the tether length to be 0, we're fully reeled in.
        void setTetherLength(double) except + # pays out or reels in the tether to this length
        double setMaxTetherSpeed(double) except + # Returns the actual payout rate set.  Will cap based on the motor & gearbox capabilities.
        void setTetherSpeed(double) except + # Also re-commands the latest commanded length
        void setTetherAccelDecel(double, double) except + # Only takes effect when you update the speed.  In meters/s^2
        double getTetherLength() except +

cdef class PyReelController:
    cdef ReelController *rc
    def __cinit__(self, string usb_port, double reel_diam_cm):
        self.rc = new ReelController(usb_port, reel_diam_cm)
    def __dealloc__(self):
        del self.rc
    def setTetherToHome(self): # consider the tether length to be 0, we're fully reeled in.
        return self.rc.setTetherToHome()
    def setTetherLength(self, double desired_length_m): # pays out or reels in the tether to this length
        return self.rc.setTetherLength(desired_length_m)
    def setMaxTetherSpeed(self, double max_tether_mps): # Returns the actual payout rate set.  Will cap based on the motor & gearbox capabilities.
        return self.rc.setMaxTetherSpeed(max_tether_mps)
    def setTetherSpeed(self, double tether_mps): # Also re-commands the latest commanded length
        return self.rc.setTetherSpeed(tether_mps)
    def setTetherAccelDecel(self, double accel_mpss, double decel_mpss): # Only takes effect when you update the speed.  In meters/s^2
        return self.rc.setTetherAccelDecel(accel_mpss, decel_mpss)
    def getTetherLength(self):
        return self.rc.getTetherLength()
