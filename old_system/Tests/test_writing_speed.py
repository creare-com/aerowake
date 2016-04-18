import datetime 
import time

t1_0 = time.clock()
t2_0 = datetime.datetime.now()

with open('output.csv', 'w') as f:
    for i in xrange(100000):
        t1 = time.clock()
        t2 = datetime.datetime.now()
        td1 = t1-t1_0
        td2 = (t2-t2_0).total_seconds()
        f.write('%.6f,%.6f\n' % (td1, td2))

print "It took", td1, "seconds to write 100000 samples."
print "Writespeed on SD card is", td1 / 100000.0 * 1000, "ms per write"

with open('/mnt/usb/output.csv', 'w') as f:
    for i in xrange(100000):
        t1 = time.clock()
        t2 = datetime.datetime.now()
        td1 = t1-t1_0
        td2 = (t2-t2_0).total_seconds()
        f.write('%.6f,%.6f\n' % (td1, td2))

print "It took", td1, "seconds to write 100000 samples."
print "Writespeed on USB is", td1 / 100000.0 * 1000, "ms per write"


