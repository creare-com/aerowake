# -*- coding: utf-8 -*-
"""
Created on Fri Sep 04 08:44:02 2015

@author: mpu
"""

import traits.api as t
import traitsui.api as tui
import inspect
import numpy as np
from pymavlink.dialects.v10 import ardupilotmega as apm

import MAVProxy
import droneapi.module.api

from ..Controller.custom_messages import set_mission_mode, mission_status

mavlink_msgs = [key[15:] for key in apm.__dict__.keys() \
                if "MAVLINK_MSG_ID_" in key]

mavlink_msgs_filt = [
 'MISSION_ACK',
 'COMMAND_INT',
 'POSITION_TARGET_LOCAL_NED',
 'SET_MODE',
 'LOG_ENTRY',
 'DEBUG',
 'NAMED_VALUE_INT',
 'GPS_GLOBAL_ORIGIN',
 'MISSION_COUNT',
 'MANUAL_CONTROL',
 'SAFETY_ALLOWED_AREA',
 'CHANGE_OPERATOR_CONTROL_ACK',
 'ATTITUDE_QUATERNION',
 'GLOBAL_POSITION_INT',
 'MAG_CAL_REPORT',
 'ATTITUDE',
 'LOG_REQUEST_LIST',
 'COMMAND_ACK',
 'GPS_INJECT_DATA',
 'DEBUG_VECT',
 'PID_TUNING',
 'RC_CHANNELS_OVERRIDE',
 'GPS2_RTK',
 'NAMED_VALUE_FLOAT',
 'RADIO_STATUS',
 'GPS_STATUS',
 'MISSION_REQUEST_LIST',
 'MISSION_WRITE_PARTIAL_LIST',
 'HIL_SENSOR',
 'LANDING_TARGET',
 'MISSION_CURRENT',
 'MISSION_ITEM',
 'PARAM_VALUE',
 'ALTITUDE',
 'ATTITUDE_QUATERNION_COV',
 'RALLY_POINT',
 'MISSION_REQUEST',
 'MISSION_ITEM_INT',
 'LOG_REQUEST_END',
 'POWER_STATUS',
 'FENCE_POINT',
 'FENCE_STATUS',
 'SET_ATTITUDE_TARGET',
 'REQUEST_DATA_STREAM',
 'DATA_STREAM',
 'SENSOR_OFFSETS',
 'LOCAL_POSITION_NED_COV',
 'PARAM_SET',
 'PING',
 'LOG_REQUEST_DATA',
 'MISSION_SET_CURRENT',
 'HOME_POSITION',
 'SET_GPS_GLOBAL_ORIGIN',
 'SET_HOME_POSITION',
 'RALLY_FETCH_POINT',
 'TIMESYNC',
 'MISSION_CLEAR_ALL',
 'MISSION_REQUEST_PARTIAL_LIST',
 'MANUAL_SETPOINT',
 'SYS_STATUS',
 'HEARTBEAT',
 'LOCAL_POSITION_NED',
 'AUTOPILOT_VERSION_REQUEST',
]

mission_status = {
    0: "TAKING_OFF",
    1: "LANDING", 
    2: "FLYING MISSION",
    3: "STOPPING MISSION",
    4: "LOITERING", 
    9: "EMERGENCY STOP",
}

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
    mission_status = t.Enum(["IN_PROGESS", 
                             "ERROR",
                             "WAITING",
                             "STOPPED",
                             "STARTING"])
    
    # GPS
    gps_altitude = t.Float(t.Undefined)
    gps_distance = t.Float(t.Undefined)
    
    # Probe
    probe_u = t.Float(label="u (m/s)")
    probe_v = t.Float(label="v (m/s)")
    probe_w = t.Float(label="w (m/s)")
    
    # ui stuff
    update_mission = t.Button("Update")
    filtered = t.Bool(True)
    send_mavlink_message = t.Button("Send")
    
    traits_view = tui.View(
        tui.Group(
                  tui.Group(
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
                            
                            tui.Item(name="gps_altitude", enabled_when='False'),
                            tui.Item(name='gps_distance', enabled_when='False'),
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
        self.mavlink_message_params = np.ones(self.mavlink_message_params.size + 1)
