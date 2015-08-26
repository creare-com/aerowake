import time
import datetime
import smbus
import numpy as np
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import signal, sys



api = local_connect()
vehAPI=api.get_vehicles()[0]

def location_callback(location):
	print "Location: " + str(vehAPI.location.lat) + "     attitude: " + str(vehAPI.attitude.pitch)

vehAPI.add_attribute_observer('attitude' , location_callback)


while True:
	print "Waiting"
	time.sleep(1)







