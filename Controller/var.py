#Name: var.py
#Author: mklinker
#Usage: Global Variables

import numpy as np

# ADC Variables

adcGain = 6144 
adcSPS = 250 
adc0Address=0x48

# Logging Variables

logN = 0

# Failsafe Variables

failsafeActive = 0  #1=failsafe active, land the uav!


# UAV Physical Properties
m = 2.135 #kg
g = 9.81  


# Position Controller Gains
kp_th = 40
kp_phi = 40

kd_th = 10
kd_phi = 10

ki_th = 0
ki_phi = 0

kp_pose = np.array([kp_th,kp_phi])
kd_pose = np.array([kd_th,kd_phi])

kp_yaw = 1
kd_yaw = 1









