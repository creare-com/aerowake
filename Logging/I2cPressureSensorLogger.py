import smbus
import time
import Adafruit_GPIO.I2C as I2C

def parse_bits(a):
    b = ''.join(['0'*(8-aa.bit_length())+ bin(aa)[2:] for aa in a])
    return b


class PressureSensor(object):
    def __init__(self, address, FSS=0.5, differential=True):
        """
        Parameters
        -----------
        address : int
            i2c sensor address. Can be specified as (for example) 0x10
        FSS : float
            Sensor full scale level. span in inH2O
        differential : bool
            If this is a differential pressure sensor (True) or a guage pressure sensor (False)
        """
        # Save attributes
        self.address = address
        self.FSS = FSS
        self.differential = differential
        if differential:
            self.OSdig = 8192
        else:
            self.OSdig = 1638

	# Open the bus
        busnum = I2C.get_default_bus()
        self._bus = smbus.SMBus(busnum)
      
    def read_p(self):
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
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS * (self.differential + 1)
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)
    def __del__(self):
        self._bus.close()

if __name__ == "__main__":
    import time
    
    # Settings
    diff_p_sensor_addresses = [
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
        0x18, 0x20,
    ]
    diff_p_sensor_desc = [ # Human-readable descriptions for CSV header row
        'Ch0  lo', 'Ch0  hi',
        'Ch1  lo', 'Ch1  hi',
        'Ch2  lo', 'Ch2  hi',
        'Ch3  lo', 'Ch3  hi',
        'Ch4  lo', 'Ch4  hi',
        'Ch5  lo', 'Ch5  hi',
        'Ch6  lo', 'Ch6  hi',
        'Ch7  lo', 'Ch7  hi',
        'Ch8  lo', 'Ch8  hi',
        'Ch9  lo', 'Ch9  hi',
        'Ch10 lo', 'Ch10 hi',
        'Ch11 lo', 'Ch11 hi',
    ]
    csv_column_names=['System time (s)', 'Time since previous line (ms)'] + diff_p_sensor_desc
    FSS_for_all_sensors = 0.5
    diff_p_sensors = [PressureSensor(addr, FSS=FSS_for_all_sensors) for addr in diff_p_sensor_addresses]
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
