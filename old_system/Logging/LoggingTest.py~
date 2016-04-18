
import time
import datetime
import smbus
import numpy as np
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time, signal, sys, datetime
from Adafruit_ADS1x15 import ADS1x15

ADS1015 = 0x00  # 12-bit ADC
gain = 6144
sps = 250  # 250 samples per second

api = local_connect()
v=api.get_vehicles()[0]

bus = smbus.SMBus(1)

def get_attitude():
	att = v.attitude
	return att

def get_location():
	loc = v.location
	return loc

print "Data Logger Initialized"

count=1

while count<100:

	adc0 = ADS1x15(ic=ADS1015,address=0x48)
	Ch00 = adc0.readADCSingleEnded(0,gain,sps)/1000
	Ch01 = adc0.readADCSingleEnded(1,gain,sps)/1000
	Ch02 = adc0.readADCSingleEnded(0,gain,sps)/1000
	Ch03 = adc0.readADCSingleEnded(1,gain,sps)/1000

	f=open('data.txt','a')
	now= datetime.datetime.now()
	timestamp = now.strftime("%H:%M:%S")
	attitude = get_attitude()
	location = get_location()
	outstr = str(count) + "  "+ str(timestamp)+ " " + str(attitude)+ " " + str(Ch00) + "\n"
	print outstr
	f.write(outstr)
	f.close()
	count=count+1

	time.sleep(.1)
