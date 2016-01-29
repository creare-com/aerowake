import time
from sensors import DlvrPressureSensor, DlvPressureSensor, TemperatureSensor

if __name__ == "__main__":
    import time
    
    # Settings
    FSP_lo = 10 #inH20, for DLVR-F50D-E2NS-C-NI3F
    FSP_hi = 1  #inH20, for DLVR-L05D-E3NS-C-NI3F
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
    probe_sensors = [DlvrPressureSensor(**set) for set in probe_sensor_settings]
    absolute_sensor = DlvPressureSensor(desc='Absolute pressure (PSIA)', addr=0x10, FSP=30)
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
            elapsed_time_reading = 0
            elapsed_time_parsing = 0
            num_readings = 0
            while True:
                probe_pressures = [] # apparently list append is fairly fast
                # Read all data as close to simultaneously as possible
                t0 = time.time()
                for sensor in probe_sensors:
                    sensor.retrieve_p()
                t1 = time.time()
                # Do all the parsing overhead afterwards
                for sensor in probe_sensors:
                    probe_pressures.append(sensor.parse_p())
                t2 = time.time()
                probe_pressure_str=','.join([str(p) for p in probe_pressures])
                if now - time_last_read_temp >= temp_read_interval_s:
                    t_str = str(temp_sensor.read_temp_c())
                    ap_str = str(absolute_sensor.read_p())
                    time_last_read_temp = now
                    log_str = '%.3f,%.3f,%s,%s,%s\n'%(now,dt*1000,probe_pressure_str,ap_str,t_str)
                else:
                    log_str = '%.3f,%.3f,%s\n'%(now,dt*1000,probe_pressure_str)
                elapsed_time_reading += (t1 - t0)
                elapsed_time_parsing += (t2 - t1)
                num_readings += 1
                dt=time.time()-now
                now = time.time()
                logfile.write(log_str)
    except KeyboardInterrupt:
        print "Caught keyboard interrupt; exiting."
        print "Spent an average of %fms on reading and an average of %fms on parsing."%((elapsed_time_reading / num_readings)*1000, (elapsed_time_parsing / num_readings)*1000)
    # except Exception as e:
        # print "Caught exception:\n%s"%repr(e)
