"""
Reel Control top object

Object hierarchy as follows (connections represent composition not inheritance):

ReelController (in ReelController.py)
  | 
PyReelController (in rc.pyx) - Note: this is added to Python's global import list by running "python setup.py build_ext --inplace"
  |
ReelController (in ReelController.hpp/cpp)
  |
EposMotorController (in EposMotorController.hpp/cpp)

"""

import math
import logging

class ReelController:
    def __init__(self, interface='USB0', reel_diam_m=0.127):
        self._N_PER_ADC_COUNT        = 0.0045203
        self._SENSOR_BASELINE_COUNTS = 7991
        self._SENSOR_DEADBAND_COUNTS = 100
        self._T_DEADBAND_N           = self._N_PER_ADC_COUNT * self._SENSOR_DEADBAND_COUNTS
        self._reel_diam_m = reel_diam_m # Need to set this first so the conversion methods to work
        self._MAX_RPM = 60
        self._MIN_RPM = 6
        self._MAX_MPS = self.tether_mps_from_reel_rpm(self._MAX_RPM)
        self._MIN_MPS = self.tether_mps_from_reel_rpm(self._MIN_RPM)
        self._L_MAX_SPEED_M = 10 # length no longer limits reel speed beyond this range
        self._T_MAX_SPEED_N = 5  # After this many newtons of force, don't limit payout rate
        self._KT_MPS_PER_N  =  self._L_MAX_SPEED_M / (self._T_MAX_SPEED_N - self._T_DEADBAND_N)
        self._KL_MPS_PER_M  = (self._MAX_MPS - self._MIN_MPS) / self._L_MAX_SPEED_M
        
        # For easier testing, create mock objects if the real ones fail.
        try:
            from TensionSensor import TensionSensor
            self._tension_sensor = TensionSensor(adc_per_ct=self._N_PER_ADC_COUNT, adc_baseline=self._SENSOR_BASELINE_COUNTS)
            self._tension_sensor.readTension()
        except:
            logging.warning("Cannot connect to tension sensor!  Will be using mock sensor instead.")
            from MockTensionSensor import MockTensionSensor
            self._tension_sensor = MockTensionSensor()

        try:
            from rc import PyReelController
            self._rc = PyReelController(interface, reel_diam_m*10)
        except:
            logging.warning("Cannot connect to motor controller!  Will be using mock motor controller instead.")
            from MockPyReelController import MockPyReelController
            self._rc = MockPyReelController()
    
    def tether_mps_from_reel_rpm(self, reel_rpm):
        return reel_rpm   * (math.pi * self._reel_diam_m / 60)
    def reel_rpm_from_tether_mps(self, tether_mps):
        return tether_mps / (math.pi * self._reel_diam_m / 60)
    
    def update(self):
        """
        Updates the maximum speed of the motor controller
        based on the tension from the tension sensor while 
        the reel spools to the correct tether length.
        Call this method frequently in your main loop.
        """
        
        current_length = self._rc.getTetherLength();
        target_length  = self._rc.getTetherTargetLength();
        
        # Update the maximum speed of the motor controller differently
        # if it's spooling out vs reeling in.  When spooling out, we
        # care about the tension, since we don't want to let line out until
        # the UAV will take up the slack.  When reeling in, we want the
        # UAV to slow down as it approaches the landing site.
        length_limited_speed  = self._KL_MPS_PER_M * current_length + self._MIN_MPS
        if current_length > target_length:
            # Reeling in
            speed_limit = min(self._MAX_MPS, length_limited_speed)
        else: # Stationary OR reeling out
            # Apply tension deadband
            tension_n = self._tension_sensor.readTension()
            if tension_n < self._T_DEADBAND_N:
                speed_limit = 0
            else:
                tension_n -= self._T_DEADBAND_N # Prevent "step" up when exiting deadband
                tension_limited_speed = self._KT_MPS_PER_N * tension_n
                speed_limit = min(self._MAX_MPS, length_limited_speed, tension_limited_speed)
        
        # Apply speed limit
        self._rc.setMaxTetherSpeed(speed_limit)
    
    def setTetherLengthM(self, tether_length_m):
        self._rc.setTetherLength(tether_length_m)
        self.update()
        
