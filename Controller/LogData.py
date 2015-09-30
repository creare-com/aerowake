#Name: LogData.py
#Author: mklinker
#Usage: Logging vehicle data

import time
import datetime

# import files
import var

def write_to_log(dataPkt):
    with open(var.log_file_name,'a') as f:
        var.logN+=1
        if var.logN ==1:
            header_text = "log N,timestamp,roll,pitch,yaw,throttle,vox,voy,voz,vth,vphi,vr,airspd,lat,lon,alt,th,phi,goal_th,goal_phi,fx,fy,fz,fix,fiy,fiz,p,r,thr,yw,s_lat,s_lon,t_dist,s_alt,s_head,s_vel_x,s_vel_y,s_vel_z \n"
            f.write(header_text)            
            
            
        now= datetime.datetime.now()
        timestamp = now.strftime("%H:%M:%S")
        outstr = str(var.logN) + ","+ str(timestamp)+","+ str(dataPkt)+ "\n"
        f.write(outstr)
        if var.logN<5:        
            print "Writing to Logfile Successful"
        
