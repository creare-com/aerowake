from droneapi.module.api import APIModule
from custom_messages import get_mission_mode, send_mission_status

class GCS(object): 
    """
    This object will poll and fill its attributes based on mavlink messages
    send from the GCS
    """
    # Tether inputs
    tether_tension = None
    tether_length = None
    tether_velocity = None
    
    # Mission inputs
    mission_mode_int = None
    mission_mode_str = ''
    
    # Flight scheduler inputs
    sweep_angle = None
    sweep_alt_max = None
    sweep_alt_min = None
    
    # ON GCS: Incoming
    # GCS Pixhawk    
    gcs_eph = None
    gcs_epv = None
    gcs_satellites_visible = None
    gcs_fix_type = None
    
    gcs_airspeed = None
    gcs_groundspeed = None
    gcs_heading = None
    gcs_velocity = None
    
    # Location inputs
    gcs_alt = None
    gcs_lat = None
    gcs_lon = None
    
    # Attitude inputs
    gcs_pitch = None
    gcs_roll = None
    gcs_yaw = None
    gcs_pitchspeed = None
    gcs_yawspeed = None
    gcs_rollspeed = None
    
    gcs_conn = None
    def __init__(self, gcs_conn):
        """
        This constructor will set up a callback to the GCS connection
        
        Parameters
        -----------
        gcs_conn : MAVLink object
            This is the connection that was opened to the GCS through telemetry
            radio on UAV
        """
        self.gcs_conn = gcs_conn
        self.gcs_conn.mav.set_callback(self.gcs_callback, self.gcs_conn)

    def gcs_callback(self, m, gcs):
        """
        This parses messages from the GCS and fills the attributes of this 
        class. 
        
        These messages are forward to the UAV pixhawk automatically through
        mavproxy.
        """
        if getattr(m, '_timestamp', None) is None:
            gcs.post_message(m)
        mtyp = m.get_type()
        if mtyp == "TETHER_STATUS":
            self.tether_length = m.length
            self.tether_tension = m.tension
            self.tether_velocity = m.velocity
        elif mtyp == "SET_MISSION_MODE":
            self.mission_mode_int = m.mode
            self.mission_mode_str = get_mission_mode.get(m.mode, '')
        elif mtyp == "SCHEDULE_SWEEP":
            self.sweep_angle = m.angle
            self.sweep_alt_start = m.altitude_start
            self.sweep_alt_end = m.altitude_end
            self.sweep_alt_step = m.altitude_step
        elif mtyp == 'GLOBAL_POSITION_INT':
            (self.gcs_lat, self.gcs_lon) = (m.lat / 1.0e7, m.lon / 1.0e7)
            self.gcs_velocity = (m.vx / 100.0, m.vy / 100.0, m.vz / 100.0)
        elif mtyp == 'GPS_RAW_INT':
            self.gcs_eph = m.eph
            self.gcs_epv = m.epv
            self.gcs_satellites_visible = m.satellites_visible
            self.gcs_fix_type = m.fix_type
        elif mtyp == "VFR_HUD":
            self.gcs_heading = m.heading
            self.gcs_alt = m.alt
            self.gcs_airspeed = m.airspeed
            self.gcs_groundspeed = m.groundspeed
        elif mtyp == "ATTITUDE":
            self.gcs_pitch = m.pitch
            self.gcs_yaw = m.yaw
            self.gcs_roll = m.roll
            self.gcs_pitchspeed = m.pitchspeed
            self.gcs_yawspeed = m.yawspeed
            self.gcs_rollspeed = m.rollspeed
   #     self.report()
    
    def report(self):
        """
        A simple little function that prints out all the modified attributes
        of the class
        """
        print "*"*80
        for key in self.__dict__.keys():
            if '__' not in key:
                print key, getattr(self, key)
        print "*"*80


class APIModuleGCSMod(APIModule):
    """
    This is the plugin class that modifies the DRONEAPI class
    
    Basically all we're doing here is adding a 'gcs' attribute which we 
    initialize using the class above. 
    """
    def __init__(self, mpstate):
        super(APIModuleGCSMod, self).__init__(mpstate)
        self.gcs = GCS(mpstate.mav_outputs[0])
        print "Modified DroneAPI loaded"
    def get_connection(self):
        """ To make the gcs available through the dronekit api, we need to
        return it when 'local_connect()' is called from the dronekit script
        
        This function is that same as the local_connect() function.
        """
        return self.api, self.gcs

def init(mpstate):
    '''initialise module'''
    return APIModuleGCSMod(mpstate)
