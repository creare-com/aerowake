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
        self._QC_PER_TURN            = 1024*4
        self._MOTOR_MAX_RPM          = 10000
        self._GEARBOX_MAX_INPUT_RPM  = 8000
        
        # Post-gearbox settings
        self._REEL_ACCEL_RPMS        = 100 # Used in the Profile, ramps up   speed at this rate
        self._REEL_DECEL_RPMS        = 100 # Used in the Profile, ramps down speed at this rate
        self._REEL_MAX_VEL_RPM       = None #100 # Set as the max RPM - profile velocity will be limited to this value.  Set to None here to compute it based on the motor.
        self._MAX_RPM                = 60 # The highest RPM commanded by the tether speed equations
        self._MIN_RPM                = 6  # The lowest  RPM commanded by the tether speed equations
        
        # Sensor settings
        self._N_PER_ADC_COUNT        = 0.0045203
        self._SENSOR_BASELINE_COUNTS = 7848
        self._SENSOR_DEADBAND_COUNTS = 100
        self._T_DEADBAND_N           = self._N_PER_ADC_COUNT * self._SENSOR_DEADBAND_COUNTS

        # Reel system settings
        self._reel_diam_m            = reel_diam_m # Need to set this first so the conversion methods to work
        self._MAX_MPS                = self.tetherMpsFromReelRpm(self._MAX_RPM) # _MAX_RPM in mps
        self._MIN_MPS                = self.tetherMpsFromReelRpm(self._MIN_RPM) # _MIN_RPM in mps
        self._L_MAX_SPEED_M          = 10 # length no longer limits reel speed beyond this range
        self._T_MAX_SPEED_N          = 5  # Above this many newtons of force, don't limit payout rate
        self._KT_MPS_PER_N           =  self._L_MAX_SPEED_M / (self._T_MAX_SPEED_N - self._T_DEADBAND_N)
        self._KL_MPS_PER_M           = (self._MAX_MPS - self._MIN_MPS) / self._L_MAX_SPEED_M
        self._home_pos_m             = 0
        
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
        if self._REEL_MAX_VEL_RPM == None:
            self._REEL_MAX_VEL_RPM = self.computeMaxTetherSpeedRpm()
        self._mc.setMaxVelocity(self._REEL_MAX_VEL_RPM)
        
        # Initialize tether system
        self.youAreHome()
        self._mc.clearFaultAndEnable() # movement may occur after this point

    # Conversion functions
    def motorPositionFromTetherLength(self, tether_length_m):
        return -(self._QC_PER_TURN * ((tether_length_m + self._home_pos_m)/ (math.pi * self._reel_diam_m)))
    def tetherLengthFromMotorPosition(self, motor_position):
        return (-motor_position / self._QC_PER_TURN) * (math.pi * self._reel_diam_m) - self._home_pos_m;
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
    
    def _recommandMotorPosition(self):
        """
        Reads the motor's target position, and tells it to go there.
        This is needed to cause a new profile to take effect, or to
        resume the motor's movement after being halted.
        Keep in mind the motor's target is the last position for 
        which it received a command - don't call this method unless
        you want the reel to resume movement and go there.
        """
        position_to_recommand = self._mc.getTargetPosition()
        logging.info("Re-commanding the tether to %fm."%self.tetherLengthFromMotorPosition(position_to_recommand))
        self._mc.moveToPosition(position_to_recommand)
    
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
            self._mc.haltMovement()
            actual_max_mps = 0
        else:
            actual_max_mps = self.setMaxTetherSpeedMps(speed_limit)
        self._recommandMotorPosition() # Causes the motor controller to move at the new speed
        logging.info("Max tether mps wound up being %f"%actual_max_mps)

    def stopMoving(self):
        self._mc.haltMovement()
    
    def setTetherLengthM(self, tether_length_m):
        self._last_commanded_m = tether_length_m
        desired_motor_position = self.motorPositionFromTetherLength(tether_length_m)
        self._mc.moveToPosition(desired_motor_position)
        self.update()
        
    def getTetherLengthM(self):
        cur_motor_position = self._mc.getPosition()
        return self.tetherLengthFromMotorPosition(cur_motor_position)
        
    def getTargetTetherLengthM(self):
        tgt_motor_position = self._mc.getTargetPosition()
        return self.tetherLengthFromMotorPosition(tgt_motor_position)
        
    def computeMaxTetherSpeedRpm(self):
        motor_limited_rpm   = self._MOTOR_MAX_RPM / self._gear_ratio
        gearbox_limited_rpm = self._GEARBOX_MAX_INPUT_RPM / self._gear_ratio
        motor_max_rpm = min(motor_limited_rpm, gearbox_limited_rpm)
        return motor_max_rpm
        
    def setMaxTetherSpeedMps(self, max_tether_mps):
        # Convert to RPM
        max_tether_rpm = self.reelRpmFromTetherMps(max_tether_mps)
        # Tone it down if necessary 
        max_tether_rpm = min(max_tether_rpm, self._REEL_MAX_VEL_RPM)
        self._mc.setPositionProfile(velocity=max_tether_mps,
            acceleration=self._REEL_ACCEL_RPMS, deceleration=self._REEL_DECEL_RPMS)
        return self.tetherMpsFromReelRpm(max_tether_rpm)

    def getMaxTetherSpeedMps(self):
        return self.tetherMpsFromReelRpm(self._mc.getPositionProfile()['velocity']);
