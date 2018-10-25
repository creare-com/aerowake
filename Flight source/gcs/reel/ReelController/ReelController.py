"""
Reel Control top object

Object hierarchy as follows (connections represent composition not inheritance):

ReelController (in ReelController.py)
  | 
PyMotorController (in PyMotorController.pyx) - Note: this is added to Python's global import list by running "python setup.py build_ext --inplace"
  |
EposMotorController (in EposMotorController.hpp/cpp)

"""
import time
import sys
import math
import logging

class ReelController:
    def __init__(self, interface='USB0', reel_diam_m=0.127):
        # Logger setup
        if 'reel_logger' in logging.Logger.manager.loggerDict:
            self._logger = logging.getLogger('reel_logger')
            print 'ReelController.py is using reel.py logger'
        else:
            self._logger = logging.getLogger('reel_logger')
            self._logger.setLevel(logging.DEBUG)
            print 'ReelController.py created its own reel_logger'

        # Motor settings
        self._QC_PER_TURN            = -1024*4 # Flip the sign - the motor considers "positive" to be the direction that retracts the tether
        self._MOTOR_MAX_RPM          = 10000
        self._GEARBOX_MAX_INPUT_RPM  = 8000
        
        # Post-gearbox settings
        self._REEL_ACCEL_RPMS        = 60 # Used in the Profile, ramps up   speed at this rate
        self._REELING_IN_DECEL_RPMS  = 150 # Used in the Profile, ramps down speed at this rate
        self._REELING_OUT_DECEL_RPMS = None # Decelerate quicker while reeling out to prevent letting the tether off the pulleys.  Set to None here to read it from the motor controller. Maximum from motor is 300.
        self._REEL_MAX_VEL_RPM       = None #100 # Set as the max RPM - profile velocity will be limited to this value.  Set to None here to compute it based on the motor.
        self._MAX_RPM                = 120 # The highest RPM commanded by the tether speed equations
        self._MIN_RPM                = 50 # The lowest  RPM commanded by the tether speed equations, changed this value from 6 to 24 on 7/31/18 after flight test 1
        self._REELING_IN_ACCEL_RPMS  = 10 # accelerate at this speed on reelin
        self._REELING_IN_MAX_RPM_NO_TENSION         = 20 # Maximum RPM when reeling in if there is not enough tension on the line. If there is enough tension, max rpm when reeling in is self._MAX_RPM.
        self._REELING_IN_TENSION_REQUIRED_N         = 3 # There must be at least this many newtons on the tension sensor when reeling in, otherwise reel in will limit max rpm
	self._REELING_IN_LENGTH_TO_IGNORE_TENSION_M = 3 # If tether length is less than this, do not set speed based on tension
        
        # Sensor settings
        self._N_PER_ADC_COUNT        = 0.0045203
        self._SENSOR_BASELINE_COUNTS = 7716 # subtracting 132 10/25/18 to zero tension sensor # subtracting 77 7/31/18 to try to offset tension zero point
        self._SENSOR_DEADBAND_COUNTS = 250 # Changed this value from 500 to 250 on 7/31/18 after flight test 1
        self._T_DEADBAND_N           = self._N_PER_ADC_COUNT * self._SENSOR_DEADBAND_COUNTS

        # Reel system settings
        self._reel_diam_m            = reel_diam_m # Need to set this first so the conversion methods to work
        self._MAX_MPS                = self.tetherMpsFromReelRpm(self._MAX_RPM) # _MAX_RPM in mps
        self._MIN_MPS                = self.tetherMpsFromReelRpm(self._MIN_RPM) # _MIN_RPM in mps
        self._REELING_IN_MAX_MPS_NO_TENSION = self.tetherMpsFromReelRpm(self._REELING_IN_MAX_RPM_NO_TENSION)
        self._L_MAX_SPEED_M          = 10 # length no longer limits reel speed beyond this range
        self._T_MAX_SPEED_N          = 5  # Above this many newtons of force, don't limit payout rate
        self._KT_MPS_PER_N           = 0.2 # 5N at 1 MPS desired #(self._L_MAX_SPEED_M / (self._T_MAX_SPEED_N - self._T_DEADBAND_N)) * (3.6/2.6) # last multiplier is a workaround to keep preious gain, added on 7/31/18 after flight test
        self._KL_MPS_PER_M           = (self._MAX_MPS - self._MIN_MPS) / self._L_MAX_SPEED_M
        self._QC_PER_M               = self._QC_PER_TURN / (math.pi * self._reel_diam_m)
        self._home_pos_m             = 0
        
        try:
            from TensionSensor import TensionSensor
            self._tension_sensor = TensionSensor(adc_per_ct=self._N_PER_ADC_COUNT, adc_baseline=self._SENSOR_BASELINE_COUNTS)
            self._tension_sensor.readTension()
        except:
            self._logger.warning("Cannot connect to tension sensor!  Will be using mock sensor instead.")
            from MockTensionSensor import MockTensionSensor
            self._tension_sensor = MockTensionSensor()
            self._tension_sensor.setTension(1)

        self._initMotorController(interface)
        
        # Configure motor controller
        self._gear_ratio = self._mc.getGearRatioNumerator() / self._mc.getGearRatioDenominator()
        if self._REELING_OUT_DECEL_RPMS == None:
            self._REELING_OUT_DECEL_RPMS = self._mc.getMaxAccelDecel()
        if self._REEL_MAX_VEL_RPM == None:
            self._REEL_MAX_VEL_RPM = self.computeMaxTetherSpeedRpm()
        print 'DECEL RPMS: ', self._REELING_OUT_DECEL_RPMS
        self._mc.setMaxVelocity(self._REEL_MAX_VEL_RPM)
        self._logger.debug("getMaxAccelDecel() = %f"%self._mc.getMaxAccelDecel())
        self._logger.debug("_REELING_OUT_DECEL_RPMS = %f"%self._REELING_OUT_DECEL_RPMS)
        self._logger.debug("_REEL_MAX_VEL_RPM = %f"%self._REEL_MAX_VEL_RPM)
        
        
        # Initialize tether system
        self.youAreHome()
        self._mc.clearFaultAndEnable() # movement may occur after this point

    def _initMotorController(self, interface):
        """
        Attempt to connect to the motor controller on the specified interface.
        Upon success, self._mc will be set to an instance of PyMotorController.
        Upon failure, self._mc will be set to an instance of MockPyMotorController.
        """
        try:
            from PyMotorController import PyMotorController, SensorType
            self._mc = PyMotorController(interface)
            
            self._mc.setOperatingMode('PROFILE_POSITION')
        except Exception as err:
            self._logger.error("Error while connecting to motor controller: " + str(err))
            self._logger.warning("Cannot connect to motor controller!  Will be using mock motor controller instead.")
            from MockPyMotorController import MockPyMotorController
            self._mc = MockPyMotorController()
        
        
    # Conversion functions
    def motorPositionFromTetherLength(self, tether_length_m):
        """ tether_length_m is considered to be relative to the home position """
        return self._QC_PER_M * (tether_length_m + self._home_pos_m)
    def tetherLengthFromMotorPosition(self, motor_position):
        """ Returns meters relative to the home position """
        return motor_position / self._QC_PER_M - self._home_pos_m;
    def tetherMpsFromReelRpm(self, reel_rpm):
        return reel_rpm   * (math.pi * self._reel_diam_m / 60)
    def reelRpmFromTetherMps(self, tether_mps):
        return tether_mps / (math.pi * self._reel_diam_m / 60)
    

    def __del__(self):
        self.stopMoving()
        del self._mc
        del self._tension_sensor
        
    def youAreHome(self,home_pos_m=None):
        """ Consider the tether's current position to be 0m """
        if home_pos_m is None:
	    self._home_pos_m = 0 # Zero this out because getTetherLengthM() returns a value relative to home
            self._home_pos_m = self.getTetherLengthM()
        else:
	    self._home_pos_m = home_pos_m
    
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
        
        dir = "--"
        mv = 'o'

        # Update the maximum speed of the motor controller differently
        # if it's spooling out vs reeling in.  When spooling out, we
        # care about the tension, since we don't want to let line out until
        # the UAV will take up the slack.  When reeling in, we want the
        # UAV to slow down as it approaches the landing site.
        length_limited_speed  = self._KL_MPS_PER_M * current_length + self._MIN_MPS
        tension_n = self.getTetherTensionN()
        if current_length > target_length:
            # Reeling in
            if current_length > self._REELING_IN_LENGTH_TO_IGNORE_TENSION_M:
                if tension_n < self._REELING_IN_TENSION_REQUIRED_N:
                    speed_limit = min(self._REELING_IN_MAX_MPS_NO_TENSION, length_limited_speed)
                else:
                    speed_limit = min(self._MAX_MPS, length_limited_speed)
            else:
                speed_limit = min(self._MAX_MPS, length_limited_speed)
            mv = 'l'
            dir = "<-"
            actual_max_mps = self._setMaxTetherSpeedMps(speed_limit, reeling_out=False, accel = self._REELING_IN_ACCEL_RPMS)
            self._recommandMotorPosition() # Causes the motor controller to move at the new speed
        else: # Stationary OR reeling out
            # Apply tension deadband
            if tension_n < self._T_DEADBAND_N:
                self.stopMoving()
                mv = 'x'
                actual_max_mps = 0
            else:
                tension_n -= self._T_DEADBAND_N # Prevent "step" up when exiting deadband
                tension_limited_speed = self._KT_MPS_PER_N * tension_n
                # speed_limit = min(self._MAX_MPS, length_limited_speed, tension_limited_speed)
                speed_limit = min(self._MAX_MPS, tension_limited_speed)
                mv = 't' if speed_limit == tension_limited_speed else ('l' if speed_limit == length_limited_speed else '-')
                dir = "->"
                actual_max_mps = self._setMaxTetherSpeedMps(speed_limit, reeling_out=True)
                self._recommandMotorPosition() # Causes the motor controller to move at the new speed
        
        status_str = dir[0] + mv + dir[1] + " %3.3fm->%3.3f @%3.8fmps %03.8fN "%(current_length, target_length, actual_max_mps, tension_n)
        self._logger.debug(status_str)

    def stopMoving(self):
        try:
            self._mc.haltMovement()
        except RuntimeError as err:
            self._logger.error("Error in _mc.haltMovement: " + str(err))
    
    def setTetherLengthM(self, tether_length_m):
        desired_motor_position = self.motorPositionFromTetherLength(tether_length_m)
        try:
            self._mc.moveToPosition(desired_motor_position)
        except RuntimeError as err:
            self._logger.error("Error in _mc.moveToPosition: " + str(err))
        self.update()
        
    def getTetherLengthM(self):
        try:
            cur_motor_position = self._mc.getPosition()
        except RuntimeError as err:
            self._logger.error("Error in _mc.getPosition: " + str(err))
            cur_motor_position = 0
        return self.tetherLengthFromMotorPosition(cur_motor_position)
        
    def getTargetTetherLengthM(self):
        try:
            tgt_motor_position = self._mc.getTargetPosition()
        except RuntimeError as err:
            self._logger.error("Error in _mc.getTargetPosition: " + str(err))
            tgt_motor_position = 0
        return self.tetherLengthFromMotorPosition(tgt_motor_position)
        
    def computeMaxTetherSpeedRpm(self):
        motor_limited_rpm   = self._MOTOR_MAX_RPM / self._gear_ratio
        gearbox_limited_rpm = self._GEARBOX_MAX_INPUT_RPM / self._gear_ratio
        motor_max_rpm = min(motor_limited_rpm, gearbox_limited_rpm)
        return motor_max_rpm
        
    def _setMaxTetherSpeedMps(self, max_tether_mps, reeling_out=True, accel=None):
	if accel is None:
            accel = self._REEL_ACCEL_RPMS
        if reeling_out: decel = self._REELING_OUT_DECEL_RPMS
        else:           decel = self._REELING_IN_DECEL_RPMS
        # Convert to RPM
        max_tether_rpm = self.reelRpmFromTetherMps(max_tether_mps)
        # Tone it down if necessary 
        max_tether_rpm = min(max_tether_rpm, self._REEL_MAX_VEL_RPM)
        if max_tether_rpm < 1:
            self._logger.debug("Cannot set max tether speed to %fRPM because it's <1."%max_tether_rpm)
            return 0
        else:
            try:
                self._mc.setPositionProfile(velocity=max_tether_rpm,
                    acceleration=accel, deceleration=decel)
            except RuntimeError as err:
                self._logger.error("Error in _mc.setPositionProfile: " + str(err))
            return self.tetherMpsFromReelRpm(max_tether_rpm)

    def getMaxTetherSpeedMps(self):
        try:
            return self.tetherMpsFromReelRpm(self._mc.getPositionProfile()['velocity']);
        except RuntimeError as err:
            self._logger.error("Error in _mc.getPositionProfile: " + str(err))
            return 0

    def getTetherTensionN(self):
        return self._tension_sensor.readTension()

    def isEnabled(self):
        try:
            return self._mc.isEnabled()
        except RuntimeError as err:
            self._logger.error("Error in _mc.isEnabled: " + str(err))
            return False

    def isFaulted(self):
        try:
            return self._mc.isFaulted()
        except RuntimeError as err:
            self._logger.error("Error in _mc.isFaulted: " + str(err))
            return False

    def clearFault(self):
        try:
            self._mc.clearFault()
        except RuntimeError as err:
            self._logger.error("Error in _mc.clearFault: " + str(err))

    def clearFaultAndEnable(self):
        if self.isFaulted():
            try:
                curr_pos = self.getTetherLengthM()
                self._mc.clearFaultAndEnable()
                self.youAreHome(home_pos_m = -curr_pos) # Reset home so current position = curr_pos
            except RuntimeError as err:
                self._logger.error("Error in _mc.clearFaultAndEnable: " + str(err))
        else:
            self._logger.info("clearFaultAndEnable called when motor controller is not faulted.")
