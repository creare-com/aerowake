# -*- coding: utf-8 -*-
"""
Created on Fri Sep 04 08:44:02 2015

@author: mpu
"""
import threading
import time
import platform
from pyface.api import GUI

import traits.api as t
import traitsui.api as tui
import inspect
import numpy as np
from pymavlink.dialects.v10 import ardupilotmega as apm
from pymavlink import mavutil

from custom_messages import set_mission_mode, mission_status

mavlink_msgs = [key[15:] for key in apm.__dict__.keys() \
                if "MAVLINK_MSG_ID_" in key]

mavlink_msgs_filt = [
 'SET_HOME_POSITION',
 'HEARTBEAT',
]

# Figuire out the number of intputs for all the mavlink messages, and also
# the function call...
#%%
mavlink_msgs_attr = {}
n_max = 0
for key in mavlink_msgs:
    try: 
        n = len(inspect.getargspec(getattr(apm.MAVLink, key.lower()
                                                       + '_send')).args)
    except:
        n = 0
    mavlink_msgs_attr[key] = {'name': key.lower() + "_send", 'n': n} 
    if n > n_max:
        n_max = n
   
#%% This is a little silly, but if we want the vehicle class to behave nicely, 
# it has to be a traited class
class Location(t.HasTraits):
    alt = t.Float
    lat = t.Float
    lon = t.Float
class Attitude(t.HasTraits):
    pitch = t.Float
    roll = t.Float
    yaw = t.Float
class Battery(t.HasTraits):
    current = t.Float()
    level = t.Float()
    voltage = t.Float()    
class PixHawk(t.HasTraits):
    modename = t.Str
    airspeed = t.Float
    groundspeed = t.Float
    armed = t.Bool
    location = t.Instance(Location)
    velocity = t.Array(shape=(3, ))
    attitude = t.Instance(Attitude)
    battery = t.Instance(Battery)
    def __init__(self, vehicle, *args, **kwargs):
        super(PixHawk, self).__init__(*args, **kwargs)
        self.modename = vehicle.mode.name
        for attr in ['airspeed', 'groundspeed', 'armed', 'velocity']:
            setattr(self, attr, getattr(vehicle, attr))
        self.location=Location(alt=vehicle.location.alt,
                               lat=vehicle.location.lat,
                               lon=vehicle.location.lon)
           
        self.attitude = Attitude(pitch=vehicle.attitude.pitch,
                                 roll=vehicle.attitude.roll,
                                 yaw=vehicle.attitude.yaw)
        self.battery = Battery(current=vehicle.battery.current,
                               level=vehicle.battery.level,
                               voltage=vehicle.battery.voltage)
                           
#%%

class GCS(t.HasTraits):
    # ON GCS: Outgoing
    mission_message = t.Enum(set_mission_mode.keys(),
                              label="Mission Type")
    sweep_angle = t.Float(label="Angle (degrees)")
    sweep_alt_max = t.Float(label="Max Altitude (m)")
    sweep_alt_min = t.Float(label="Min Altitude (m)")
    
    mavlink_message = t.Enum(mavlink_msgs, label="Mavlink Message")    
    mavlink_message_filt = t.Enum(mavlink_msgs_filt, label="Mavlink Message")    
    mavlink_message_params = t.Array(label="params")
    
    # ON GCS: Incoming
    # Tether
    tether_length = t.Float(t.Undefined, label='Length (m)')
    tether_tension = t.Float(t.Undefined, label='Tension (N)')
    tether_velocity = t.Float(t.Undefined, label="Velocity")
    
    # ON DRONE: Incoming
    # Mission Status
    mission_status = t.Enum(mission_status.keys())
        
    # Probe
    probe_u = t.Float(label="u (m/s)")
    probe_v = t.Float(label="v (m/s)")
    probe_w = t.Float(label="w (m/s)")
    
    # Vehicle inputs
    modename = t.Str
    airspeed = t.Float
    groundspeed = t.Float
    armed = t.Bool
    location = t.Instance(Location)
    velocity = t.Array(shape=(3, ))
    
    # Location inputs
    alt = t.Float
    lat = t.Float
    lon = t.Float    
    
    # Attitude inputs
    pitch = t.Float
    roll = t.Float
    yaw = t.Float    
    
    # Battery Inputs 
    current = t.Float()
    level = t.Float()
    voltage = t.Float()    
    
    # Vehicle connection
    uav = t.Any()
    polling = t.Bool()
    uav_msg_thread = t.Instance(threading.Thread)
    send_mavlink_message = t.Button("Send")
    show_errors = t.Bool(True)
    mav_error = t.Int(0)
    port = t.Str()
    baud = t.Int()

    
    
    # ui stuff
    update_mission = t.Button("Update")
    filtered = t.Bool(True)
    
    
    traits_view = tui.View(
        tui.Group(
                  tui.Group(
                            tui.Item(name="mission_status", enabled_when='False'),
                            tui.Item(name="mission_message"),
                            tui.Item(name="sweep_angle", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="sweep_alt_max", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="sweep_alt_min", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="update_mission"),
                            tui.Item("_"), 
                            tui.Item("filtered"),
                            tui.Item("mavlink_message", visible_when='filtered==False'),
                            tui.Item("mavlink_message_filt", visible_when='filtered'),
                            tui.Item("mavlink_message_params"),
                            tui.Item("send_mavlink_message"),
                            tui.Item("_"), 
                            tui.Item(name="tether_tension", enabled_when='False'),
                            tui.Item(name="tether_length", enabled_when='False'),
                            tui.Item(name="tether_velocity", enabled_when='False'),
                            orientation="vertical",
                            show_border=True,
                            label="On GCS"
                            ),
                  tui.Group(
                            tui.Item(name="modename", enabled_when='False'),
                            tui.Item(name="airspeed", enabled_when='False'),
                            tui.Item(name="groundspeed", enabled_when='False'),
                            tui.Item(name='armed', enabled_when='False'),
                            tui.Item(name='alt', enabled_when='False'),
                            tui.Item(name='lat', enabled_when='False'),
                            tui.Item(name='lon', enabled_when='False'),
                            tui.Item(name='velocity', enabled_when='False'),
                            tui.Item(name='pitch', enabled_when='False'),
                            tui.Item(name='roll', enabled_when='False'),
                            tui.Item(name='yaw', enabled_when='False'),
                            tui.Item(name='current', enabled_when='False'),
                            tui.Item(name='level', enabled_when='False'),
                            tui.Item(name='voltage', enabled_when='False'),
                            tui.Item("_"),
                            tui.Item(name='probe_u', enabled_when='False'),
                            tui.Item(name='probe_v', enabled_when='False'),
                            tui.Item(name='probe_w', enabled_when='False'),
                            orientation='vertical',
                            show_border=True,
                            label="Incoming"
                            ),
                  orientation='horizontal')
    )
    
    def _send_mavlink_message_fired(self):
        print "firing"
        
    def setup_link(self, port, baud=56700):
        self.uav = mavutil.mavlink_connection(port, baud)
        self.port = port
        self.baud = baud
        
    def poll_uav(self):
        self.polling = True
        def worker():
            # Make sure we are connected
            m = self.uav
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, 0)
            print("Waiting for heartbeat from %s" % m.address)
            self.uav.wait_heartbeat()
            i = 0
            while self.polling:
                print "Polling round", i
                i += 1
                try:
                    s = m.recv(16*1024)
                except Exception:
                    time.sleep(0.1)
                # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
                # cause it to try and reconnect
                if len(s) == 0:
                    time.sleep(0.1)
            
                if 'windows' in platform.architecture()[-1].lower():
                    # strip nsh ansi codes
                    s = s.replace("\033[K","")
            
                if m.first_byte:
                    m.auto_mavlink_version(s)
                msgs = m.mav.parse_buffer(s)
                if msgs:
                    for msg in msgs:
                        if getattr(m, '_timestamp', None) is None:
                            m.post_message(msg)
                        if msg.get_type() == "BAD_DATA":
                            if self.show_errors:
                                print "MAV error: %s" % msg
                            self.mav_error += 1
                        else:
                            self.parse_msg(msg)
            print "Polling STopped"
            self.polling = False
        self.uav_msg_thread = threading.Thread(target=worker)
        self.uav_msg_thread.start()
        
    def parse_msg(self, m):
        print "Parsing Message"
        typ = m.get_type()
        if typ == 'GLOBAL_POSITION_INT':
            (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            (self.vx, self.vy, self.vz) = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
        elif typ == 'GPS_RAW':
            pass # better to just use global position int
            # (self.lat, self.lon) = (m.lat, m.lon)
            # self.__on_change('location')
        elif typ == 'GPS_RAW_INT':
            # (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.eph = m.eph
            self.epv = m.epv
            self.satellites_visible = m.satellites_visible
            self.fix_type = m.fix_type
        elif typ == "VFR_HUD":
            self.heading = m.heading
            self.alt = m.alt
            self.airspeed = m.airspeed
            self.groundspeed = m.groundspeed
        elif typ == "ATTITUDE":
            self.pitch = m.pitch
            self.yaw = m.yaw
            self.roll = m.roll
            self.pitchspeed = m.pitchspeed
            self.yawspeed = m.yawspeed
            self.rollspeed = m.rollspeed
        elif typ == "SYS_STATUS":
            self.voltage = m.voltage_battery
            self.current = m.current_battery
            self.level = m.battery_remaining
        elif typ == "HEARTBEAT":
            pass
#        elif typ in ["WAYPOINT_CURRENT", "MISSION_CURRENT"]:
#            self.last_waypoint = m.seq
#        elif typ == "RC_CHANNELS_RAW":
#            def set(chnum, v):
#                '''Private utility for handling rc channel messages'''
#                # use port to allow ch nums greater than 8
#                self.rc_readback[str(m.port * 8 + chnum)] = v
#
#            set(1, m.chan1_raw)
#            set(2, m.chan2_raw)
#            set(3, m.chan3_raw)
#            set(4, m.chan4_raw)
#            set(5, m.chan5_raw)
#            set(6, m.chan6_raw)
#            set(7, m.chan7_raw)
#            set(8, m.chan8_raw)
#        elif typ == "MOUNT_STATUS":
#            self.mount_pitch = m.pointing_a / 100
#            self.mount_roll = m.pointing_b / 100
#            self.mount_yaw = m.pointing_c / 100
#            self.__on_change('mount')
#        elif typ == "RANGEFINDER":
#            self.rngfnd_distance = m.distance
#            self.rngfnd_voltage = m.voltage
#            self.__on_change('rangefinder')
        print "Parsing Message DONE"
    def __delete__(self):
        print 'Deleting'
        self.uav.close()
            
            
if __name__ == "__main__":
    gcs = GCS()
    gcs.setup_link("COM8")
    gcs.poll_uav()
    gcs.configure_traits()
#    gcs.polling = False