from droneapi.module.api import APIModule

class GCS(object): 
    tether_tension = None
    tether_length = None
    mission_command = None
    mission_parameters = None

    gcs_conn = None
    def __init__(self, gcs_conn):
        self.gcs_conn = gcs_conn
        self.gcs_conn.mav.set_callback(self.gcs_callback, self.gcs)

    def gcs_callback(self, m, gcs):
        if getattr(m, '_timestamp', None) is None:
            gcs.post_message(m)
        # print "GCS MESSAGE:", m  # This actually works!

class APIModuleGCSMod(APIModule):
    def __init__(self, mpstate):
        super(APIModuleGCSMod, self).__init__(mpstate)
        self.gcs = GCS(mpstate.mav_outputs[0])
        # set up callback for gcs messages
    #    self.gcs.mav.set_callback(self.gcs_callback, self.gcs)
        print "Modified DroneAPI loaded"
    #def gcs_callback(self, m, gcs):
    #    if getattr(m, '_timestamp', None) is None:
    #        gcs.post_message(m)
    #    # print "GCS MESSAGE:", m  # This actually works!
    def get_connection(self):
        return self.api, self.gcs

def init(mpstate):
    '''initialise module'''
    return APIModuleGCSMod(mpstate)
