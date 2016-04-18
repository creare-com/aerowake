#import time
#import datetime
#import smbus
#import numpy as np
#from droneapi.lib import VehicleMode
#from pymavlink import mavutil
#import signal, sys



import numpy as np

def get_distance(): #Tested separately. Gives the 2D GPS position distance from the ship to UAV
    act1 =  [42.3578720 ,-71.0979608 ]
    act2 = [ 42.3579384 , -71.0977609 ]
    R = 6371229
    dlat = act1[0]-act2[0]
    dlon = act1[1]-act2[1]
    a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2
    distance = R *2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))
    return distance #[meters]


print get_distance()

