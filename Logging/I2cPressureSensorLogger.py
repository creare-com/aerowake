import smbus
import time
import Adafruit_GPIO.I2C as I2C

def parse_bits(a):
    b = ''.join(['0'*(8-aa.bit_length())+ bin(aa)[2:] for aa in a])
    return b


class PressureSensor(object):
    def __init__(self, addr, FSS=0.5, OSdig=1638, desc='Pressure Sensor'):
        """
        Parameters
        -----------
        address : int
            i2c sensor address. Can be specified as (for example) 0x10
        FSS : float
            Sensor full scale level span in the same units as the sensor output (either inH20 or psi)
        OSdig : int
            Digital offset of sensor word
        desc : string
            Human-readable description to identify this sensor
        """
        # Save attributes
        self.address = addr
        self.FSS = FSS
        self.OSdig = OSdig
        self.description = desc

	# Open the bus
        busnum = I2C.get_default_bus()
        self._bus = smbus.SMBus(busnum)
    
    def get_desc():
        return self.description
    
    def read_p(self):
        """
        Returns
        -------
        pressure : float
            The pressure indicated by the sensor, in the same units that FSS was provided to __init__ earlier
        """
        # t0 = time.time()
        a = self._bus.read_i2c_block_data(self.address, 0, 2)
        # t1 = time.time()
        b = parse_bits(a)
        # t2 = time.time()
        p = self.parse_p(b)
        # t3 = time.time()
        # print 'pressure read: %d, %d, %d'%(100000*(t1-t0), 100000*(t2-t1), 100000*(t3-t2))
        return p
    def read_pt(self):
        a = self._bus.read_i2c_block_data(self.address, 0, 4)
        b = parse_bits(a)
        return self.parse_pt(b)
    def parse_pt(self, b):
        return self.parse_p(b), self.parse_t(b)
    def parse_p(self, b):
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)
    def __del__(self):
        self._bus.close()

class DlvrPressureSensor(PressureSensor):
    def __init__(self, addr, FSP=0.5, differential=True, desc='DLVR Series Pressure Sensor'):#If this is a differential pressure sensor (True) or a gage pressure sensor (False)
        if differential:
            OSdig=8192
        else:
            OSdig=1638
        FSS = FSP * (differential + 1)
        super(DlvrPressureSensor, self).__init__(OSdig=OSdig, addr=addr, FSS=FSS, desc=desc)

class DlvPressureSensor(PressureSensor):
    def __init__(self, addr, FSP=0.5, desc='DLV Series Pressure Sensor'):
        super(DlvPressureSensor, self).__init__(OSdig=1638, addr=addr, FSS=FSP, desc=desc)
        
if __name__ == "__main__":
    import time
    
    # Settings

    FSP_lo = 10 #inH20, for DLVR-F50D-E2NS-C-NI3F
    FSP_hi = 1  #inH20, for DLVR-L05D-E3NS-C-NI3F
    # Descriptions will be used for column names, so the units are listed here too
    probe_sensor_settings = [
        {'desc':'Ch0  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch0  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch1  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch1  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch2  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch2  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch3  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch3  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch4  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch4  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch5  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch5  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch6  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch6  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch7  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch7  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch8  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch8  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch9  lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch9  hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch10 lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch10 hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
        {'desc':'Ch11 lo (inH2O)', 'addr':0x18, 'FSP':FSP_lo}, {desc:'Ch11 hi (inH2O)', 'addr':0x20, 'FSP':FSP_hi}, 
    ]
    probe_sensors = [DlvrPressureSensor(**set) for set in settings]
    absolute_sensor = DlvPressureSensor(addr=0x18, 
    csv_column_names=['System time (s)', 'Time since previous line (ms)'] + [sensor.get_desc() for sensor in probe_sensors]
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
                pressures = [] # apparently list append is fairly fast
                for sensor in diff_p_sensors:
                    p = sensor.read_p()
                    # p,t = sensor.read_pt()
                    pressures.append(p)
                pressure_str=','.join([str(p) for p in pressures])
                if now - time_last_read_temp >= temp_read_interval_s:
                    #TODO: read these sensors and log them
                    # print "Reading and logging temperature and absolute pressure. %.3f"%(dt * 1000)
                    t = 0
                    ap = 0
                    time_last_read_temp = now
                else:
                    t = None
                    ap = None
                dt=time.time()-now
                now = time.time()
                log_str = '%.3f,%.3f,%s\n'%(now,dt*1000,pressure_str)
                logfile.write(log_str)
    except KeyboardInterrupt:
        print "Caught keyboard interrupt; exiting."
    except Exception as e:
        print "Caught exception:\n%s"%repr(e)
        pass
