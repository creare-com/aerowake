import datetime
import time

class airprobe_main:
    def __init__(self,dt):

        # For easier testing, create mock objects if the real ones fail.
        try:
            from sensors import DlvrPressureSensor, DlvPressureSensor, TemperatureSensor
        except:
            print("Cannot import sensors.  Using mock objects instead for easier testing.")
            from mock_sensors import DlvrPressureSensor, DlvPressureSensor, TemperatureSensor



        # USER CONFIGURABLE SETTINGS
        self.dt_des = dt
        self.TEMP_READ_INTERVAL_S = 1.0 # Read temperature and absolute pressure every second

        FSP_lo = 0.5 #inH20, for DLVR-F50D-E2NS-C-NI3F
        FSP_hi = 2.0 #inH20, for DLVR-L05D-E3NS-C-NI3F
        # Descriptions will be used for column names, so the units are listed here too
        probe_sensor_settings = [
            {'desc':'Ch0  lo (inH2O)', 'addr':0x20, 'FSP':FSP_lo}, {'desc':'Ch0  hi (inH2O)', 'addr':0x30, 'FSP':FSP_hi}, 
            {'desc':'Ch1  lo (inH2O)', 'addr':0x21, 'FSP':FSP_lo}, {'desc':'Ch1  hi (inH2O)', 'addr':0x31, 'FSP':FSP_hi}, 
            {'desc':'Ch2  lo (inH2O)', 'addr':0x22, 'FSP':FSP_lo}, {'desc':'Ch2  hi (inH2O)', 'addr':0x32, 'FSP':FSP_hi}, 
            {'desc':'Ch3  lo (inH2O)', 'addr':0x23, 'FSP':FSP_lo}, {'desc':'Ch3  hi (inH2O)', 'addr':0x33, 'FSP':FSP_hi}, 
            {'desc':'Ch4  lo (inH2O)', 'addr':0x24, 'FSP':FSP_lo}, {'desc':'Ch4  hi (inH2O)', 'addr':0x34, 'FSP':FSP_hi}, 
            {'desc':'Ch5  lo (inH2O)', 'addr':0x25, 'FSP':FSP_lo}, {'desc':'Ch5  hi (inH2O)', 'addr':0x35, 'FSP':FSP_hi}, 
            {'desc':'Ch6  lo (inH2O)', 'addr':0x26, 'FSP':FSP_lo}, {'desc':'Ch6  hi (inH2O)', 'addr':0x36, 'FSP':FSP_hi}, 
            {'desc':'Ch7  lo (inH2O)', 'addr':0x27, 'FSP':FSP_lo}, {'desc':'Ch7  hi (inH2O)', 'addr':0x37, 'FSP':FSP_hi}, 
            {'desc':'Ch8  lo (inH2O)', 'addr':0x28, 'FSP':FSP_lo}, {'desc':'Ch8  hi (inH2O)', 'addr':0x38, 'FSP':FSP_hi}, 
            {'desc':'Ch9  lo (inH2O)', 'addr':0x29, 'FSP':FSP_lo}, {'desc':'Ch9  hi (inH2O)', 'addr':0x39, 'FSP':FSP_hi}, 
            {'desc':'Ch10 lo (inH2O)', 'addr':0x2a, 'FSP':FSP_lo}, {'desc':'Ch10 hi (inH2O)', 'addr':0x3a, 'FSP':FSP_hi}, 
            {'desc':'Ch11 lo (inH2O)', 'addr':0x2b, 'FSP':FSP_lo}, {'desc':'Ch11 hi (inH2O)', 'addr':0x3b, 'FSP':FSP_hi}, 
        ]
        self._probe_sensors = [DlvrPressureSensor(**set) for set in probe_sensor_settings]
        self._absolute_sensor = DlvPressureSensor(desc='Absolute pressure (PSIA)', addr=0x40, FSP=30)
        self._temp_sensor = TemperatureSensor(desc='Ambient temperature (C)', addr=0x18)
        csv_column_names=['System time (s)', 'Time since previous line (ms)'] \
            + [sensor.get_desc() for sensor in self._probe_sensors] + [self._absolute_sensor.get_desc(), self._temp_sensor.get_desc()]
        self._temp_read_interval_s = 1.0 # Read temperature and absolute pressure every second
        logfile_name = time.strftime('pressure_log_%Y-%m-%d_%H%M%S.csv',time.localtime())
        self._logfile = open(logfile_name, 'wc')
        self._logfile.write('"'+'","'.join(csv_column_names)+'"\n')
        self._logfile.flush()
        self._now = time.time()
        self._dt = 0
        self._elapsed_time_reading = 0
        self._elapsed_time_parsing = 0
        self._num_readings = 0
        self._time_last_read_temp = time.time() - self.TEMP_READ_INTERVAL_S;

################################################################################
################################################################################

#This will get called at 200hz or whatever rate you want it to, as specified by the airprobe_run().

    def run(self):
        probe_pressures = [] # apparently list append is fairly fast
        # Read all data as close to simultaneously as possible
        t0 = time.time()
        for sensor in self._probe_sensors:
            sensor.retrieve_p()
        t1 = time.time()
        # Do all the parsing overhead afterwards
        for sensor in self._probe_sensors:
            probe_pressures.append(sensor.parse_p())
        t2 = time.time()
        probe_pressure_str=','.join([str(p) for p in probe_pressures])
        if self._now - self._time_last_read_temp >= self.TEMP_READ_INTERVAL_S:
            t_str = str(self._temp_sensor.read_temp_c())
            ap_str = str(self._absolute_sensor.read_p())
            self._time_last_read_temp = self._now
            log_str = '%.3f,%.3f,%s,%s,%s\n'%(self._now,self._dt*1000,probe_pressure_str,ap_str,t_str)
        else:
            log_str = '%.3f,%.3f,%s\n'%(self._now,self._dt*1000,probe_pressure_str)
        self._elapsed_time_reading += (t1 - t0)
        self._elapsed_time_parsing += (t2 - t1)
        self._num_readings += 1
        self._dt = time.time() - self._now
        self._now = time.time()
        self._logfile.write(log_str)
        
        return []# At some point it might be useful to report a summary of the airprobe data, but we don't currently.
