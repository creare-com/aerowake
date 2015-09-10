from droneapi.module.api import APIModule
from custom_messages import get_mission_mode, send_mission_status

class GCS(object): 
    tether_tension = None
    tether_length = None
    tether_velocity = None
    mission_mode_int = None
    mission_mode_str = ''
    
    sweep_angle = None
    sweep_alt_max = None
    sweep_alt_min = None

    gcs_conn = None
    def __init__(self, gcs_conn):
        self.gcs_conn = gcs_conn
        self.gcs_conn.mav.set_callback(self.gcs_callback, self.gcs_conn)

    def gcs_callback(self, m, gcs):
        if getattr(m, '_timestamp', None) is None:
            gcs.post_message(m)
        mtyp = m.get_type()
        if mtyp == "TETHER_STATUS":
            self.tether_length = m.tether_length
            self.tether_tension = m.tether_tension
            self.tether_velocity = m.tether_velocity
        elif mtyp == "SET_MISSION_MODE":
            self.mission_mode_int = m.mode
            self.mission_mode_str = get_mission_mode.get(m.mode, '')
        elif mtyp == "SCHEDULE_SWEEP":
            self.sweep_angle = m.angle
            self.sweep_alt_max = m.altitude_max
            self.sweep_alt_min = m.altitude_min
        self.report()
    
    def report(self):
         print "*"*80
         for key in self.__dict__.keys():
             if '__' not in key:
                 print key, getattr(self, key)
         print "*"*80
         #print "GCS MESSAGE:", m  # This actually works!

class APIModuleGCSMod(APIModule):
    def __init__(self, mpstate):
        super(APIModuleGCSMod, self).__init__(mpstate)
        self.gcs = GCS(mpstate.mav_outputs[0])
        print "Modified DroneAPI loaded"
    def get_connection(self):
        return self.api, self.gcs

def init(mpstate):
    '''initialise module'''
    return APIModuleGCSMod(mpstate)
