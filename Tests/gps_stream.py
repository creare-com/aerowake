


import datetime 
import time
import serial
import pynmea2
import gps
from gpsd_poller import GpsPoller

t1_0 = time.clock()
t2_0 = datetime.datetime.now()

#sudo gpsd /dev/ttyUSB1 -F /var/run/gpsd.sock
gpsd_ses = gps.gps("localhost","2947")
gpsd_ses.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

while True:
    try:
        report = gpsd_ses.next()
        
        if report['class'] == 'TPV':
            if hasattr(report, 'lat'):
                print report.lat,'  ',report.lon
            if hasattr(report, 'time'):
                print report.time
    except KeyError:
        pass
    except KeyboardInterrupt:
        quit()
    except StopIteration:
        gpsd_ses = None
        print "GPSD has terminated"



print 'lol done'

ser.open()

while 1:
    ser.write("rofl")
    print ser.read()
    time.sleep(.1)






