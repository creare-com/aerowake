"""
Reel Control top object

Object hierarchy as follows (connections represent composition not inheritance):

ReelController (in ReelController.py)
  | 
PyMotorController (in PyMotorController.pyx) - Note: this is added to Python's global import list by running "python setup.py build_ext --inplace"
  |
EposMotorController (in EposMotorController.hpp/cpp)

"""

import math
import logging

class ReelController:
    def __init__(self, interface='USB0', reel_diam_m=0.127):
        # Motor settings
        self._QC_PER_TURN = 1024*4;
        self._MOTOR_MAX_RPM = 10000;
        self._GEARBOX_MAX_INPUT_RPM = 8000;
        
        # Sensor settings
        self._N_PER_ADC_COUNT        = 0.0045203
        self._SENSOR_BASELINE_COUNTS = 7848
        self._SENSOR_DEADBAND_COUNTS = 100
        self._T_DEADBAND_N           = self._N_PER_ADC_COUNT * self._SENSOR_DEADBAND_COUNTS

        # Reel system settings
        self._reel_diam_m            = reel_diam_m # Need to set this first so the conversion methods to work
        self._MAX_RPM                = 60
        self._MIN_RPM                = 6
        self._MAX_MPS                = self.tetherMpsFromReelRpm(self._MAX_RPM)
        self._MIN_MPS                = self.tetherMpsFromReelRpm(self._MIN_RPM)
        self._L_MAX_SPEED_M          = 10 # length no longer limits reel speed beyond this range
        self._T_MAX_SPEED_N          = 5  # After this many newtons of force, don't limit payout rate
        self._KT_MPS_PER_N           =  self._L_MAX_SPEED_M / (self._T_MAX_SPEED_N - self._T_DEADBAND_N)
        self._KL_MPS_PER_M           = (self._MAX_MPS - self._MIN_MPS) / self._L_MAX_SPEED_M
        self._home_pos_m             = 0
        self._motor_is_halted        = False
        self._last_commanded_m       = 0
        
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
            from PyMotorController import PyMotorController, SensorType
            self._mc = PyMotorController(interface)
        except:
            logging.warning("Cannot connect to motor controller!  Will be using mock motor controller instead.")
            from MockPyMotorController import MockPyMotorController
            self._mc = MockPyMotorController()
        # Configure motor controller
        self._gear_ratio = self._mc.getGearRatioNumerator() / self._mc.getGearRatioDenominator()
        self.youAreHome()
        self._mc.clearFaultAndEnable() # movement may occur after this point

    # Conversion functions
    def motorPositionFromTetherLength(self, tether_length_m):
        return self._QC_PER_TURN * (-tether_length_m / (math.pi * self._reel_diam_m))
    def tetherLengthFromMotorPosition(self, motor_position):
        return (-motor_position / self._QC_PER_TURN) * (math.pi * self._reel_diam_m);
    def tetherMpsFromReelRpm(self, reel_rpm):
        return reel_rpm   * (math.pi * self._reel_diam_m / 60)
    def reelRpmFromTetherMps(self, tether_mps):
        return tether_mps / (math.pi * self._reel_diam_m / 60)
    

    def __del__(self):
        self.stopMoving()
        del self._mc
        del self._tension_sensor
        
    def youAreHome(self):
        """ Consider the tether's current position to be 0m """
        self._home_pos_m = self.getTetherLengthM()
        self._last_commanded_m = 0
        self.update()
    
    def update(self):
        """
        Updates the maximum speed of the motor controller
        based on the tension from the tension sensor while 
        the reel spools to the correct tether length.
        Call this method frequently in your main loop.
        """
        
        current_length = self.getTetherLengthM()
        target_length  = self.getTargetTetherLengthM()
        
        # Update the maximum speed of the motor controller differently
        # if it's spooling out vs reeling in.  When spooling out, we
        # care about the tension, since we don't want to let line out until
        # the UAV will take up the slack.  When reeling in, we want the
        # UAV to slow down as it approaches the landing site.
        length_limited_speed  = self._KL_MPS_PER_M * current_length + self._MIN_MPS
        if current_length > target_length:
            # Reeling in
            logging.info("Reeling in because %fm > %fm, length limits us to %fmps"%(current_length, target_length, length_limited_speed))
            speed_limit = min(self._MAX_MPS, length_limited_speed)
        else: # Stationary OR reeling out
            # Apply tension deadband
            tension_n = self._tension_sensor.readTension()
            if tension_n < self._T_DEADBAND_N:
                logging.info("Within deadband because %fN < %fN"%(tension_n, self._T_DEADBAND_N))
                speed_limit = 0
            else:
                tension_n -= self._T_DEADBAND_N # Prevent "step" up when exiting deadband
                tension_limited_speed = self._KT_MPS_PER_N * tension_n
                logging.info("Reeling out, length limits us to %fmps, tension limits us to %fmps"%(length_limited_speed, tension_limited_speed))
                speed_limit = min(self._MAX_MPS, length_limited_speed, tension_limited_speed)
        
        # Apply speed limit
        logging.info("Changing speed limit from %fmps to %fmps"%(self.getMaxTetherSpeedMps(),speed_limit))
        if speed_limit == 0:
            #self._mc.haltMovement()
#            self._motor_is_halted = True
            speed_limit = 0.01
#        else:
#            position_to_recommand = None # If the motor was halted, need to give it the target location again
#            if self._motor_is_halted:
#                position_to_recommand = self._mc.getTargetPosition()
        actual_max_mps = self.setMaxTetherSpeedMps(speed_limit)
#            if position_to_recommand != None:
#                logging.info("Recommanding motor to position %f"%position_to_recommand)
#                self._mc.moveToPosition(position_to_recommand)
#                self._motor_is_halted = False
        logging.info("Max tether mps wound up being %f"%actual_max_mps)

    def stopMoving(self):
        self._mc.haltMovement()
    
    def setTetherLengthM(self, tether_length_m):
        self._last_commanded_m = tether_length_m
        desired_motor_position = self.motorPositionFromTetherLength(tether_length_m + self._home_pos_m)
        self._mc.moveToPosition(desired_motor_position)
        self.update()
        
    def getTetherLengthM(self):
        cur_motor_position = self._mc.getPosition()
        return self.tetherLengthFromMotorPosition(cur_motor_position) - self._home_pos_m
        
    def getTargetTetherLengthM(self):
        tgt_motor_position = self._mc.getTargetPosition()
        return self.tetherLengthFromMotorPosition(tgt_motor_position) - self._home_pos_m
        
    def setMaxTetherSpeedMps(self, max_tether_mps):
        max_payout_rpm = self.reelRpmFromTetherMps(max_tether_mps);
        if max_payout_rpm * self._gear_ratio > self._MOTOR_MAX_RPM:
            max_payout_rpm = self._MOTOR_MAX_RPM / self._gear_ratio
        if max_payout_rpm * self._gear_ratio > self._GEARBOX_MAX_INPUT_RPM:
            max_payout_rpm = self._GEARBOX_MAX_INPUT_RPM / self._gear_ratio
        logging.info("Setting reel max RPM=%f"%(max_payout_rpm))
        self._mc.setMaxVelocity(max_payout_rpm);
        return self.tetherMpsFromReelRpm(max_payout_rpm);

    def getMaxTetherSpeedMps(self):
        max_payout_rpm = self._mc.getMaxVelocity();
        return self.tetherMpsFromReelRpm(max_payout_rpm);
