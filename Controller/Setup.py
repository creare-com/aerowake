#Name: Setup.py
#Author: mklinker
#Usage: Initialize hardware, setup 

import time
import datetime
import smbus
import numpy as np
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time, signal, sys, datetime
from Adafruit_ADS1x15 import ADS1x15
## Import Files
import var

def InitializeADCHardware():
    print "ADC Hardware Initialized"
    ADS1015 = 0x00  # 12-bit ADC
    bus = smbus.SMBus(1)
    adc0 = ADS1x15(ic=ADS1015,address=var.adc0Address)
    return adc0


