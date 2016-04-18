from pymavlink import mavutil
from pymavlink.dialects.v10.ardupilotmega import MAVLink_bad_data
pix = mavutil.mavlink_connection("/dev/ttyAMA0", 115200)  # The pixhawk
master = mavutil.mavlink_connection("/dev/ttyUSB0", 57600)  # The telemetry Radio
import time

t0 = time.clock()
td = time.clock() - t0
msglist = []
while td < 10:
    # If there is a valid pixhawk message, forward that to the GCS
    msg = pix.recv_msg()
    if msg and not isinstance(msg, MAVLink_bad_data):
        msglist.append("PIX: " + str(msg))
        master.mav.send(msg)
    # If there is a valid GCS message, forward that to the pixhawk
    msg2 = master.recv_msg()
    if msg2 and not isinstance(msg2, MAVLink_bad_data):
        msglist.append("GCS :" + str(msg2))
        pix.mav.send(msg2)
    
    td = time.clock() - t0

with open("msg.log", "w") as fid:
    fid.write('\n'.join(msglist))

