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
from pymavlink.dialects.v10 import gcs_pixhawk as gph
from pymavlink import mavutil

from custom_messages import set_mission_mode, mission_status

# Populate the dropdown of available messages that can be sent via the GUI
mavlink_msgs = [key[15:] for key in gph.__dict__.keys() \
                if "MAVLINK_MSG_ID_" in key]
mavlink_msgs.sort()
mavlink_msgs_filt = [
 'SET_HOME_POSITION',
 'HEARTBEAT',
 'SET_MODE',
]
mavlink_msgs_filt.sort()

# Figuire out the number of intputs for all the mavlink messages, and also
# the function call (so we can display that for the user)
mavlink_msgs_attr = {}
n_max = 0
for key in mavlink_msgs:
    try: 
        args = inspect.getargspec(getattr(gph.MAVLink, key.lower()
                                                       + '_send')).args
    except:
        args = []
    n = len(args)
    mavlink_msgs_attr[key] = {'name': key.lower() + "_send",
                              'n': n,
                              'args': args} 
    if n > n_max:
        n_max = n
                             
#%%

class GCS(t.HasTraits):
    """
    This is the ground control station GUI class. 
    
    For usage, for example if telemetry radio is on COM8 and the GCS Pixhawk is
    on COM7:
    >>> gcs = GCS()
    >>> gcs.setup_uav_link("COM8")
    >>> gcs.poll_uav()
    >>> gcs.setup_gcs_link("COM7")
    >>> gcs.poll_gcs()
    >>> gcs.configure_traits()
    >>> gcs.close()
    """
    # Connections
    dialect = t.Str("gcs_pixhawk")
    show_errors = t.Bool(True)
    
    # ON GCS: Outgoing
    mission_message = t.Enum(set_mission_mode.keys(),
                              label="Mission Type")
    autopilot_mode = t.Bool(False)                              
    sweep_angle = t.Float(label="Angle (degrees)")
    sweep_alt_start = t.Float(label="Start Altitude (m)")
    sweep_alt_end = t.Float(label="End Altitude (m)")
    sweep_alt_step = t.Float(label="Number of Altitude Steps")
    
    mavlink_message = t.Enum(mavlink_msgs, label="Mavlink Message")    
    mavlink_message_filt = t.Enum(mavlink_msgs_filt, label="Mavlink Message")    
    mavlink_message_params = t.Str(label="params")
    mavlink_message_args = t.Str(', '.join(mavlink_msgs_attr[mavlink_msgs_filt[0]]['args'][1:]), label="Arguments")
    
    # ON GCS: Incoming
    # Tether
    tether_length = t.Float(t.Undefined, label='Length (m)')
    tether_tension = t.Float(t.Undefined, label='Tension (N)')
    tether_velocity = t.Float(t.Undefined, label="Velocity")
    
    # ON GCS: Incoming
    # GCS Pixhawk    
    gcs_eph = t.Float(t.Undefined)
    gcs_epv = t.Float(t.Undefined)
    gcs_satellites_visible = t.Int(t.Undefined)
    gcs_fix_type = t.Int(t.Undefined)
    
    gcs_airspeed = t.Float(t.Undefined)
    gcs_groundspeed = t.Float(t.Undefined)
    gcs_heading = t.Float(t.Undefined)
    gcs_velocity = t.Array(shape=(3, ))
    
    # Location inputs
    gcs_alt = t.Float(t.Undefined)
    gcs_lat = t.Float(t.Undefined)
    gcs_lon = t.Float(t.Undefined)  
    
    # Attitude inputs
    gcs_pitch = t.Float(t.Undefined)
    gcs_roll = t.Float(t.Undefined)
    gcs_yaw = t.Float(t.Undefined)
    gcs_pitchspeed = t.Float(t.Undefined)
    gcs_yawspeed = t.Float(t.Undefined)
    gcs_rollspeed = t.Float(t.Undefined)
    
    # Battery Inputs 
    gcs_current = t.Float(t.Undefined)
    gcs_level = t.Float(t.Undefined)
    gcs_voltage = t.Float(t.Undefined)   
    
    # GCS connectinos
    gcs = t.Any(t.Undefined)
    gcs_polling = t.Bool(False)
    gcs_msg_thread = t.Instance(threading.Thread)    
    gcs_error = t.Int(0)
    gcs_port = t.Str(t.Undefined)
    gcs_baud = t.Int(t.Undefined)


    # ON DRONE: Incoming
    # Mission Status
    mission_status = t.Enum(mission_status.keys())
        
    # Probe
    probe_u = t.Float(t.Undefined, label="u (m/s)")
    probe_v = t.Float(t.Undefined, label="v (m/s)")
    probe_w = t.Float(t.Undefined, label="w (m/s)")
    
    # Vehicle inputs
    uav_modename = t.Str(t.Undefined)
    uav_armed = t.Bool(t.Undefined)
    uav_eph = t.Float(t.Undefined)
    uav_epv = t.Float(t.Undefined)
    uav_satellites_visible = t.Int(t.Undefined)
    uav_fix_type = t.Int(t.Undefined)

    uav_airspeed = t.Float(t.Undefined)
    uav_groundspeed = t.Float(t.Undefined)
    uav_heading = t.Float(t.Undefined)
    uav_velocity = t.Array(shape=(3, ))
    
    # Location inputs
    uav_alt = t.Float(t.Undefined)
    uav_lat = t.Float(t.Undefined)
    uav_lon = t.Float(t.Undefined)    
    
    # Attitude inputs
    uav_pitch = t.Float(t.Undefined)
    uav_roll = t.Float(t.Undefined)
    uav_yaw = t.Float(t.Undefined)
    uav_pitchspeed = t.Float(t.Undefined)
    uav_yawspeed = t.Float(t.Undefined)
    uav_rollspeed = t.Float(t.Undefined)
    
    # Battery Inputs 
    uav_current = t.Float(t.Undefined)
    uav_level = t.Float(t.Undefined)
    uav_voltage = t.Float(t.Undefined)    
    
    # Vehicle Connections
    uav = t.Any(t.Undefined)
    uav_polling = t.Bool(False)
    uav_msg_thread = t.Instance(threading.Thread)
    uav_error = t.Int(0)
    uav_port = t.Str(t.Undefined)
    uav_baud = t.Int(t.Undefined)
    
    # GCS connectinos
    gcs = t.Any(t.Undefined)
    gcs_polling = t.Bool(False)
    gcs_msg_thread = t.Instance(threading.Thread)    
    gcs_error = t.Int(0)
    gcs_port = t.Str(t.Undefined)
    gcs_baud = t.Int(t.Undefined)
    
    # ui Buttons and display groups
    update_mission = t.Button("Update")
    send_mavlink_message = t.Button("Send")
    filtered = t.Bool(True)
    
    group_input = tui.Group(
                            tui.Item(name="mission_status", enabled_when='False'),
                            tui.Item(name="autopilot_mode"),
                            tui.Item(name="mission_message"),
                            tui.Item(name="sweep_angle", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="sweep_alt_start", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="sweep_alt_end", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),
                            tui.Item(name="sweep_alt_step", 
                                     visible_when='mission_message=="SCHEDULE_SWEEP"'),                                     
                            tui.Item(name="update_mission"),
                            tui.Item("_"), 
                            tui.Item("filtered"),
                            tui.Item("mavlink_message", visible_when='filtered==False'),
                            tui.Item("mavlink_message_filt", visible_when='filtered'),
                            tui.Item("mavlink_message_args", enabled_when='False',
                                     editor=tui.TextEditor(), height=-40 ),
                            tui.Item("mavlink_message_params"),
                            tui.Item("send_mavlink_message"),
                            tui.Item("_"), 
                            tui.Item(name="tether_tension", enabled_when='False'),
                            tui.Item(name="tether_length", enabled_when='False'),
                            tui.Item(name="tether_velocity", enabled_when='False'),
                            tui.Item("_"), 
                            orientation="vertical",
                            show_border=True,
                            label="On GCS"
                            )
    group_uav = tui.Group(
                            tui.Item(name="uav_modename", enabled_when='False'),
                            tui.Item(name="uav_airspeed", enabled_when='False'),
                            tui.Item(name="uav_groundspeed", enabled_when='False'),
                            tui.Item(name='uav_armed', enabled_when='False'),
                            tui.Item(name='uav_alt', enabled_when='False'),
                            tui.Item(name='uav_lat', enabled_when='False'),
                            tui.Item(name='uav_lon', enabled_when='False'),
                            tui.Item(name='uav_velocity', enabled_when='False'),
                            tui.Item(name='uav_pitch', enabled_when='False'),
                            tui.Item(name='uav_roll', enabled_when='False'),
                            tui.Item(name='uav_yaw', enabled_when='False'),
                            tui.Item(name='uav_current', enabled_when='False'),
                            tui.Item(name='uav_level', enabled_when='False'),
                            tui.Item(name='uav_voltage', enabled_when='False'),
                            tui.Item("_"),
                            tui.Item(name='probe_u', enabled_when='False'),
                            tui.Item(name='probe_v', enabled_when='False'),
                            tui.Item(name='probe_w', enabled_when='False'),
                            orientation='vertical',
                            show_border=True,
                            label="Incoming"
                            )
    group_gcs = tui.Group(
                            tui.Item(name="gcs_airspeed", enabled_when='False'),
                            tui.Item(name="gcs_groundspeed", enabled_when='False'),
                            tui.Item(name='gcs_alt', enabled_when='False'),
                            tui.Item(name='gcs_lat', enabled_when='False'),
                            tui.Item(name='gcs_lon', enabled_when='False'),
                            tui.Item(name='gcs_velocity', enabled_when='False'),
                            tui.Item(name='gcs_pitch', enabled_when='False'),
                            tui.Item(name='gcs_roll', enabled_when='False'),
                            tui.Item(name='gcs_yaw', enabled_when='False'),
                            tui.Item(name='gcs_current', enabled_when='False'),
                            tui.Item(name='gcs_level', enabled_when='False'),
                            tui.Item(name='gcs_voltage', enabled_when='False'),
                            orientation='vertical',
                            show_border=True,
                            label="GCS"
                            )
    traits_view = tui.View(
        tui.Group(
                  group_input,
                  tui.Group(
                            group_uav,
                            group_gcs,
                            orientation='horizontal'
                  ),
                  orientation='vertical'),
        resizable=True
    )
    
    def _autopilot_mode_changed(self):
        self.uav.mav.set_pilot_mode_send(self.autopilot_mode)
        if self.autopilot_mode:
            self.uav.set_mode(0)
    
    def _update_mission_fired(self):
        """ This will fire when the update_mission button is clicked
        
        In that case we send one of our custom MAVLINK messages, either
        set_mission_mode or schedule_sweep
        
        """
        mode = set_mission_mode[self.mission_message]
        if mode >=0:
            self.uav.mav.set_mission_mode_send(mode)
        else:
            self.uav.mav.schedule_sweep_send(self.sweep_angle, 
                                             self.sweep_alt_start,
                                             self.sweep_alt_end,
                                             self.sweep_alt_step)

    def _mavlink_message_changed(self):
        """ This will fire when the dropdown is changed
        """
        self.mavlink_message_args = ', '.join(mavlink_msgs_attr[self.mavlink_message]['args'][1:])
    
    def _mavlink_message_filt_changed(self):
        """ This will fire when the filtered dropdown is changed
        """
        self.mavlink_message_args = ', '.join(mavlink_msgs_attr[self.mavlink_message_filt]['args'][1:])
    
    def _send_mavlink_message_fired(self):
        """ This will fire when the send_mavlink_message button is clicked
        
        In that case we pass on the mavlink message that the user is trying
        to send. 
        """
        func = mavlink_msgs_attr[self.mavlink_message]['name']
        args = [float(m) for m in self.mavlink_message_params.split(',')]
        getattr(self.uav.mav, func)(*args)
        
    def setup_uav_link(self, uav_port, uav_baud=56700):
        """
        This sets up the connection to the UAV. 
        
        Parameters
        -----------
        uav_port : str
            Serial port where UAV is connected (via telemetry radio)
        uav_baud: int, optional
            The baud rate. Default is 56700
        """
        mavutil.set_dialect(self.dialect)
        self.uav = mavutil.mavlink_connection(uav_port, uav_baud)
        self.uav_port = uav_port
        self.uav_baud = uav_baud
    
    def setup_gcs_link(self, gcs_port, gcs_baud=115200):
        """
        This sets up the connection to the GCS Pixhawk. 
        
        Parameters
        -----------
        uav_port : str
            Serial port where GCS Pixhawk is connected (via usb cable)
        uav_baud: int, optional
            The baud rate. Default is 115200
        """
        mavutil.set_dialect(self.dialect)
        self.gcs = mavutil.mavlink_connection(gcs_port, gcs_baud)
        self.gcs_port = gcs_port
        self.gcs_baud = gcs_baud
        
    def poll_uav(self):
        """
        This runs a new thread that listens for messages from the UAV and
        parses them for the GCS
        """
        self.uav_polling = True
        def worker():
            # Make sure we are connected
            m = self.uav
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, 0)
            print("Waiting for heartbeat from %s" % m.address)
            self.uav.wait_heartbeat()
            print "Found Heardbeat, continuing"

            i = 0
            while self.uav_polling:
#                print "uav_polling round", i
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
                            self.uav_error += 1
                        else:
                            self.parse_uav_msg(msg)
            print "uav_polling Stopped"
            self.uav_polling = False
        self.uav_msg_thread = threading.Thread(target=worker)
        self.uav_msg_thread.start()

    def poll_gcs(self):
        """
        This runs a new thread that listens for messages from the GCS Pixhawk
        and parses them for the GCS, it also forwards relevant messages to the
        UAV
        """
        self.gcs_polling = True
        def worker():
            # Make sure we are connected
            m = self.gcs
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, 0)
            print("Waiting for heartbeat from %s" % m.address)
            self.gcs.wait_heartbeat()
            print "Found Heardbeat, continuing"
            i = 0
            while self.gcs_polling:
#                print "gcs_polling round", i
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
                            self.gcs_error += 1
                        else:
                            self.parsefwd_gcs_msg(msg)
            print "gcs_polling Stopped"
            self.gcs_polling = False
        self.gcs_msg_thread = threading.Thread(target=worker)
        self.gcs_msg_thread.start()
        
    def parse_uav_msg(self, m):
        """
        This parses a message received from the UAV and stores the values
        in the class attributes so that the GUI will update
        """
#        print "Parsing Message"
        typ = m.get_type()
        if typ == 'GLOBAL_POSITION_INT':
            (self.uav_lat, self.uav_lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.uav_velocity = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
        elif typ == 'GPS_RAW':
            pass # better to just use global position int
            # (self.lat, self.lon) = (m.lat, m.lon)
            # self.__on_change('location')
        elif typ == 'GPS_RAW_INT':
            # (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.uav_eph = m.eph
            self.uav_epv = m.epv
            self.uav_satellites_visible = m.satellites_visible
            self.uav_fix_type = m.fix_type
        elif typ == "VFR_HUD":
            self.uav_heading = m.heading
            self.uav_alt = m.alt
            self.uav_airspeed = m.airspeed
            self.uav_groundspeed = m.groundspeed
        elif typ == "ATTITUDE":
            self.uav_pitch = m.pitch
            self.uav_yaw = m.yaw
            self.uav_roll = m.roll
            self.uav_pitchspeed = m.pitchspeed
            self.uav_yawspeed = m.yawspeed
            self.uav_rollspeed = m.rollspeed
        elif typ == "SYS_STATUS":
            self.uav_voltage = m.voltage_battery
            self.uav_current = m.current_battery
            self.uav_level = m.battery_remaining
        elif typ == "HEARTBEAT":
            pass
#        print "Parsing Message DONE"

    def fwd_msg_to_uav(self, m):
        """This forwards messages from the GCS Pixhawk to the UAV if there is
        a UAV connected"""
        if self.uav is not t.Undefined:
            self.uav.write(m.get_msgbuf())

    def parsefwd_gcs_msg(self, m):
        """
        This parses a message received from the GCS Pixhawk, stores the values
        in the class attributes so that the GUI will update, and forwards 
        relevant messages to the UAV
        """
#        print "Parsing Message"
        typ = m.get_type()
        if typ == 'GLOBAL_POSITION_INT':
            (self.gcs_lat, self.gcs_lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.gcs_velocity = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
            # Forward message
            self.fwd_msg_to_uav(m)
        elif typ == 'GPS_RAW':
            # better to just use global position int
            # (self.lat, self.lon) = (m.lat, m.lon)
            # self.__on_change('location')
            # Forward message
            self.fwd_msg_to_uav(m)
        elif typ == 'GPS_RAW_INT':
            # (self.lat, self.lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.gcs_eph = m.eph
            self.gcs_epv = m.epv
            self.gcs_satellites_visible = m.satellites_visible
            self.gcs_fix_type = m.fix_type
            # Forward message
            self.fwd_msg_to_uav(m)
        elif typ == "VFR_HUD":
            self.gcs_heading = m.heading
            self.gcs_alt = m.alt
            self.gcs_airspeed = m.airspeed
            self.gcs_groundspeed = m.groundspeed
            # Forward message
            self.fwd_msg_to_uav(m)
        elif typ == "ATTITUDE":
            self.gcs_pitch = m.pitch
            self.gcs_yaw = m.yaw
            self.gcs_roll = m.roll
            self.gcs_pitchspeed = m.pitchspeed
            self.gcs_yawspeed = m.yawspeed
            self.gcs_rollspeed = m.rollspeed
            # Forward message
            self.fwd_msg_to_uav(m)
        elif typ == "SYS_STATUS":
            self.gcs_voltage = m.voltage_battery
            self.gcs_current = m.current_battery
            self.gcs_level = m.battery_remaining
        elif typ == "HEARTBEAT":
            # Forward message
            self.fwd_msg_to_uav(m)
#        print "Parsing Message DONE"

    def close(self, *args, **kwargs):
        """
        This closes down the serial connections and stop the GUI polling
        """
        print 'Closing down connection'
        try:
            self.uav_polling = False
            self.uav.close()
        except:
            pass
        try:
            self.gcs_polling = False
            self.gcs.close()
        except:
            pass
            
            
if __name__ == "__main__":
    gcs = GCS()
   # gcs.setup_uav_link("COM9", 56700)
   # gcs.poll_uav()
#    gcs.setup_gcs_link("COM7", 115200)
#    gcs.poll_gcs()
    gcs.configure_traits()
#    gcs.close()