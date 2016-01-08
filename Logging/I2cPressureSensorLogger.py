import time
from I2cSensor import I2cSensor

class PressureSensor(I2cSensor):
    """
    This class handles math and interactions common among All Sensors pressure sensors.
    """
    def __init__(self, addr, FSS=0.5, OSdig=1638, desc='Pressure Sensor'):
        """
        
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        FSS : float
            Sensor full scale level span in the same units as the sensor output (either inH20 or psi)
        OSdig : int
            Digital offset of sensor word
        desc : string
            Human-readable description to identify this sensor
        """
        super(PressureSensor, self).__init__(addr=addr, desc=desc)
        # Save attributes
        self.FSS = FSS
        self.OSdig = OSdig
    
    def read_p(self):
        """
        Returns
        -------
        pressure : float
            The pressure indicated by the sensor, in the same units that FSS was provided to __init__ earlier
        """
        bits = self.read_bit_data(0, 2)
        p = self.parse_p(bits)
        return p
    def read_pt(self):
        bits = self.read_bit_data(0, 4)
        return self.parse_pt(bits)
    def parse_pt(self, b):
        return self.parse_p(b), self.parse_t(b)
    def parse_p(self, b):
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)

class DlvrPressureSensor(PressureSensor):
    """
    This class reads from the DLVR Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, differential=True, desc='DLVR Series Pressure Sensor'):#If this is a differential pressure sensor (True) or a gage pressure sensor (False)
        if differential:
            OSdig=8192
        else:
            OSdig=1638
        FSS = FSP * (differential + 1)
        super(DlvrPressureSensor, self).__init__(OSdig=OSdig, addr=addr, FSS=FSS, desc=desc)

class DlvPressureSensor(PressureSensor):
    """
    This class reads from the DLV Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, desc='DLV Series Pressure Sensor'):
        super(DlvPressureSensor, self).__init__(OSdig=1638, addr=addr, FSS=FSP, desc=desc)

class TemperatureSensor(I2cSensor):
    def __init__(self, addr, desc='Temperature Sensor'):
        """
        Specifically the Microchip MCP9808 digital temperature sensor.
        
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        desc : string
            Human-readable description to identify this sensor
        """
        super(TemperatureSensor, self).__init__(addr=addr, desc=desc)
    
    def read_t(self):
        """
        Returns
        -------
        pressure : float
            The temperature, in degrees C, of the sensor die
        """
        bits = self.read_bit_data(0, 2)
        t = self.parse_t(bits)
        return p
    def parse_t(self, b):
        abs_t = int(b[4:], 2) * 2**-4 # Temperature is represented in 0.0625 degree C increments
        sign_bit = b[3] # 0 for temps >= 0C, 1 otherwise
        return abs_t if sign_bit == 0 else -abs_t
    def __del__(self):
        self._bus.close()

if __name__ == "__main__":
    import time
    
    # Settings
    FSP_lo = 10 #inH20, for DLVR-F50D-E2NS-C-NI3F
    FSP_hi = 1  #inH20, for DLVR-L05D-E3NS-C-NI3F
    # Descriptions will be used for column names, so the units are listed here too
    probe_sensor_settings = [
        {'desc':'Ch0  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch0  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch1  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch1  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch2  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch2  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch3  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch3  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch4  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch4  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch5  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch5  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch6  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch6  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch7  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch7  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch8  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch8  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch9  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch9  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch10 lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch10 hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch11 lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {'desc':'Ch11 hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
    ]
    probe_sensors = [DlvrPressureSensor(**set) for set in probe_sensor_settings]
    absolute_sensor = DlvPressureSensor(desc='Absolute pressure (PSIA)', addr=0x18, FSP=30)
    temp_sensor = TemperatureSensor(desc='Ambient temperature (C)', addr=0x18)
    csv_column_names=['System time (s)', 'Time since previous line (ms)'] \
        + [sensor.get_desc() for sensor in probe_sensors] + [absolute_sensor.get_desc(), temp_sensor.get_desc()]
    logfile_name = 'pressure_log.csv'
    temp_read_interval_s = 1.0 # Read temperature and absolute pressure every second
    
    # Main loop
    try:
        with open(logfile_name, 'wc') as logfile:
            logfile.write('"'+'","'.join(csv_column_names)+'"\n')
            time_last_read_temp = time.time() - temp_read_interval_s;
            now = time.time()
            dt = 0
            while True:
                probe_pressures = [] # apparently list append is fairly fast
                for sensor in probe_sensors:
                    p = sensor.read_p()
                    probe_pressures.append(p)
                probe_pressure_str=','.join([str(p) for p in probe_pressures])
                if now - time_last_read_temp >= temp_read_interval_s:
                    t_str = str(temp_sensor.read_t())
                    ap_str = str(absolute_sensor.read_p())
                    time_last_read_temp = now
                    log_str = '%.3f,%.3f,%s,%s,%s\n'%(now,dt*1000,probe_pressure_str,ap_str,t_str)
                else:
                    log_str = '%.3f,%.3f,%s\n'%(now,dt*1000,probe_pressure_str)
                dt=time.time()-now
                now = time.time()
                logfile.write(log_str)
    except KeyboardInterrupt:
        print "Caught keyboard interrupt; exiting."
    # except Exception as e:
        # print "Caught exception:\n%s"%repr(e)
