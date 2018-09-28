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
import math
import logging

class ReelController:
    def __init__(self, interface='USB0', reel_diam_m=0.127):
        # Motor settings
        self._QC_PER_TURN            = -1024*4 # Flip the sign - the motor considers "positive" to be the direction that retracts the tether
        self._MOTOR_MAX_RPM          = 10000
        self._GEARBOX_MAX_INPUT_RPM  = 8000
        
        # Post-gearbox settings
        self._REEL_ACCEL_RPMS        = 60 # Used in the Profile, ramps up   speed at this rate
        self._REELING_IN_DECEL_RPMS  = 100 # Used in the Profile, ramps down speed at this rate
        self._REELING_OUT_DECEL_RPMS = None # Decelerate quicker while reeling out to prevent letting the tether off the pulleys.  Set to None here to read it from the motor controller.
        self._REEL_MAX_VEL_RPM       = None #100 # Set as the max RPM - profile velocity will be limited to this value.  Set to None here to compute it based on the motor.
        self._MAX_RPM                = 120 # The highest RPM commanded by the tether speed equations
        self._MIN_RPM                = 12  # The lowest  RPM commanded by the tether speed equations, changed this value from 6 to 24 on 7/31/18 after flight test 1
        
        # Sensor settings
        self._N_PER_ADC_COUNT        = 0.0045203
        self._SENSOR_BASELINE_COUNTS = 7848  # subtracting 77 7/31/18 to try to offset tension zero point
        self._SENSOR_DEADBAND_COUNTS = 250 # Changed this value from 500 to 250 on 7/31/18 after flight test 1
        self._T_DEADBAND_N           = self._N_PER_ADC_COUNT * self._SENSOR_DEADBAND_COUNTS

        # Reel system settings
        self._reel_diam_m            = reel_diam_m # Need to set this first so the conversion methods to work
        self._MAX_MPS                = self.tetherMpsFromReelRpm(self._MAX_RPM) # _MAX_RPM in mps
        self._MIN_MPS                = self.tetherMpsFromReelRpm(self._MIN_RPM) # _MIN_RPM in mps
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
            # self._logger.warning("Cannot connect to tension sensor!  Will be using mock sensor instead.")
            from MockTensionSensor import MockTensionSensor
            self._tension_sensor = MockTensionSensor()
            self._tension_sensor.setTension(1)

        try:
            from PyMotorController import PyMotorController, SensorType
            self._mc = PyMotorController(interface)
        except:
            # self._logger.warning("Cannot connect to motor controller!  Will be using mock motor controller instead.")
            from MockPyMotorController import MockPyMotorController
            self._mc = MockPyMotorController()
        
        # Configure motor controller
        self._mc.setOperatingMode('PROFILE_POSITION')
        self._gear_ratio = self._mc.getGearRatioNumerator() / self._mc.getGearRatioDenominator()
        if self._REELING_OUT_DECEL_RPMS == None:
            self._REELING_OUT_DECEL_RPMS = self._mc.getMaxAccelDecel()
        if self._REEL_MAX_VEL_RPM == None:
            self._REEL_MAX_VEL_RPM = self.computeMaxTetherSpeedRpm()
        self._mc.setMaxVelocity(self._REEL_MAX_VEL_RPM)
        # self._logger.info("getMaxAccelDecel() = %f"%self._mc.getMaxAccelDecel())
        # self._logger.info("_REELING_OUT_DECEL_RPMS = %f"%self._REELING_OUT_DECEL_RPMS)
        # self._logger.info("_REEL_MAX_VEL_RPM = %f"%self._REEL_MAX_VEL_RPM)
        
        
        # Initialize tether system
        self.youAreHome()
        self._mc.clearFaultAndEnable() # movement may occur after this point

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
        
    def youAreHome(self):
        """ Consider the tether's current position to be 0m """
        self._home_pos_m = 0 # Zero this out because getTetherLengthM() returns a value relative to home
        self._home_pos_m = self.getTetherLengthM()
    
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
            speed_limit = min(self._MAX_MPS, length_limited_speed)
            mv = 'l'
            dir = "<-"
            actual_max_mps = self._setMaxTetherSpeedMps(speed_limit, reeling_out=False)
            self._recommandMotorPosition() # Causes the motor controller to move at the new speed
        else: # Stationary OR reeling out
            # Apply tension deadband
            if tension_n < self._T_DEADBAND_N:
                self._mc.haltMovement()
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
        # self._logger.info(status_str)

    def stopMoving(self):
        self._mc.haltMovement()
    
    def setTetherLengthM(self, tether_length_m):
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
        
    def _setMaxTetherSpeedMps(self, max_tether_mps, reeling_out=True):
        if reeling_out: decel = self._REELING_OUT_DECEL_RPMS
        else:           decel = self._REELING_IN_DECEL_RPMS
        # Convert to RPM
        max_tether_rpm = self.reelRpmFromTetherMps(max_tether_mps)
        # Tone it down if necessary 
        max_tether_rpm = min(max_tether_rpm, self._REEL_MAX_VEL_RPM)
        if max_tether_rpm < 1:
            # self._logger.debug("Cannot set max tether speed to %fRPM because it's <1."%max_tether_rpm)
            return 0
        else:
            self._mc.setPositionProfile(velocity=max_tether_rpm,
                acceleration=self._REEL_ACCEL_RPMS, deceleration=decel)
            return self.tetherMpsFromReelRpm(max_tether_rpm)

    def getMaxTetherSpeedMps(self):
        return self.tetherMpsFromReelRpm(self._mc.getPositionProfile()['velocity']);

    def getTetherTensionN(self):
        return self._tension_sensor.readTension()
