#Name: LogData.py
#Author: mklinker
#Usage: Logging vehicle data

import time
import datetime

# import files
import var

def write_to_log(dataPkt):
    with open('log_file1.txt','a') as f:
        var.logN+=1
        now= datetime.datetime.now()
        timestamp = now.strftime("%H:%M:%S")
        outstr = str(var.logN) + " "+ str(timestamp)+" "+ str(dataPkt)+ "\n"
        f.write(outstr)
        print "Wrote to Logfile"
        
