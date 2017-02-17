#!/usr/bin/env python2

import numpy as np
import imp
import math
import datetime
from feedforward import feed_forward # Tether feed forward model. Might have robustness issues? 
from referencecommand import reference_command # Tether reference location calculator
from tetherforce import tether_force # Basic tether model, but robust

class pose_controller_class:
    def __init__(self,dt):

        # State Information
        self.uav_coord = [0,0]  	# GPS Coordinates of UAV [lat,lon] from pixhawk (DD.DDDDDDD)
        self.uav_pose = [0,0,0]		# UAV Position [theta,phi,r] (radians)
        self.uav_vel = [0,0,0]		# UAV velocity [x,y,z] from pixhawk (m/s)
        self.uav_alt = 0			# UAV Alt from pixhawk (m)
        self.uav_heading = 0		# UAV Heading (degrees)

        self.gcs_coord = [0,0]   	# GPS Coordinates of GCS [lat,lon] from pixhawk (DD.DDDDDD)
        self.gcs_vel = [0,0,0]		# GCS Velocity [x,y,z] from pixhawk (m/s)
        self.gcs_alt = 0			# GCS Altitude from pixhawk (m)
        self.gcs_heading = 0		# GCS Heading (degrees)
        self.L = 0		            # GCS Tether Length (m)
        self.gcs_tether_tension = 0 # GCS Tether Tension (newtons)

        self.uav_voltage = 0            # UAV Voltage
        self.uav_current = 0            # UAV Current (milli-amps)

        # Inputs 
        self.goal_pose = [1.57,0,10]	# UAV Goal Position [theta,phi,r] (radians)
        self.goal_mode = 3          # G_AUTO=0,  G_TAKEOFF=1,  G_LAND=2

        # # Outputs
        # self.goal_attitude = [0,0] 	# UAV Goal attitude [roll, pitch] (radians)
        # self.goal_throttle = 0		# UAV Goal Throttle (PWM Signal)
        # self.goal_heading = 0 		# UAV Goal Heading (degrees) 

        # Memory Items for Control
        self.control_c = 0
        self.dt = dt

        self.e_phi = 0
        self.e_th = 0
        self.e_r = 0
        self.e_phi_int = 0
        self.e_th_int = 0 

        self.log_n = 0
        self.human_time = 0
        self.log_file_name = 'log_generic.csv'


        # Controller Gains P D I
        # self.k_phi =[1.2, 2.0,  1.0] 
        # self.k_th  =[1,   2.0,  1.0]
        # self.k_r   =[.5,   3] 

        self.k_phi =[1.0, .1,  0] 
        self.k_th  =[1.0, .1,  0]
        self.k_r   =[0.5, .1    ] 

        self.SMART_TETHER = True

        self.uav_weight = 2*9.81 #weight of the UAV in Newtons. 




##!!!!!!!!!!!!!!!!!!!!!!!!!! Helper Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def saturate(self,val,lower,upper):
        if val>upper:
            val=upper
        if val<lower:
            val=lower
        return val

##!!!!!!!!!!!!!!!!!!!!!!!! Geometry Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    #Gives the 2D GPS position distance from the ship to UAV-- distance along the ground. 
    def get_distance(self): #Tested. 
        act1 = self.uav_coord
        act2 = self.gcs_coord
        #act1 =  [42.3578720 ,-71.0979608 ]
        #act2 = [ 42.3579384 , -71.0977609 ]
        R = 6371229
        dlat = act1[0]-act2[0]
        dlon = act1[1]-act2[1]
        a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2
        distance = R *2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))
        return distance #[meters]

    # Gives the ideal taut-tether length from the ship to the UAV. 
    def get_diagonal_distance(self): #Tested. 
        xy_dist = self.get_distance()
        z_dist = self.uav_alt-self.gcs_alt
        diag_dist = ( xy_dist**2 + z_dist**2 )**0.5
        return diag_dist #[meters]

    # Gives the bearing angle from North from the uav to ship. 
    def get_bearing(self): #Tested. 
        act1 = self.uav_coord #uav first
        act2 = self.gcs_coord #gcs second
        dlon=act1[1]-act2[1] #ship - uav
        arg1= np.sin(dlon*np.pi/180) * np.cos(act2[0]*np.pi/180) 
        arg2= np.cos(act1[0]*np.pi/180) * np.sin(act2[0]*np.pi/180) - np.sin(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * np.cos(dlon*np.pi/180)        
        g_rel_ang =  -np.arctan2( arg1, arg2)
        #if g_rel_ang<0:
        #    g_rel_ang=g_rel_ang+2*np.pi
        bearing=g_rel_ang

        return bearing#[radians, somehow]

    def get_relative_angles(self): #Tested.
        # Theta First:
        xy_dist = self.get_distance()
        z_dist = self.uav_alt-self.gcs_alt
        theta = np.arctan2(xy_dist,z_dist)
        # Phi Calc
        phi=self.gcs_heading-self.get_bearing()
        if phi>np.pi:
            phi=phi-2*np.pi
        r = self.get_diagonal_distance()
        self.uav_pose=[theta,phi,r]
        return [theta,phi,r]

    def sph_to_cart(self,th,phi,r):
        x = r*np.sin(th)*np.cos(phi)
        y = r*np.sin(th)*np.sin(phi)
        z = r*cos(phi)
        return [x,y,z]

    def eul2quat(self,roll,pitch,yaw):

        cr2 = np.cos(roll*0.5)
        cp2 = np.cos(pitch*0.5)
        cy2 = np.cos(yaw*0.5)
        sr2 = np.sin(roll*0.5)
        sp2 = np.sin(pitch*0.5)
        sy2 = np.sin(yaw*0.5)

        q1 = cr2*cp2*cy2 + sr2*sp2*sy2
        q2 = sr2*cp2*cy2 - cr2*sp2*sy2
        q3 = cr2*sp2*cy2 + sr2*cp2*sy2
        q4 = cr2*cp2*sy2 - sr2*sp2*cy2

        return [q1,q2,q3,q4]


    def set_goal(self,g_th,g_phi,L):
        
        self.L = L
        if self.SMART_TETHER:
            data_out = reference_command(g_th,g_phi,L)
            self.goal_pose = [g_th,g_phi,data_out[2]] # Theta, Phi, R_ref and sets self.L 
        if not self.SMART_TETHER:
            self.goal_pose = [g_th,g_phi,L]
        return None

    def special_att_control(self,roll,pitch,thr_cmd): # Returns quaternion for proper yaw but specified roll/pitch. 
        quat = self.eul2quat(roll,pitch,self.get_bearing())
        return [quat[0],quat[1],quat[2],quat[3],thr_cmd]       


##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Estimation Functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def get_drag_vector(self):
        return [0,0,0]

    def smart_tether_ff(self,th,phi,r,r_ref,L):
        data_out = feed_forward(phi,th,r,r_ref,L)
        Fx = data_out[0]
        Fy = data_out[1]
        Fz = data_out[2]
        return [Fx,Fy,Fz]

    def basic_tether_ff(self,th,phi,L):
        data_out = tether_force(phi,th,L)    
        Fx = data_out[0]
        Fy = data_out[1]
        Fz = data_out[2]
        return [Fx,Fy,Fz]

## !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Logging !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def write_to_log(self,dataPkt):
        with open(self.log_file_name,'a') as f:
            self.log_n+=1
            if self.log_n ==1:
                header = "self.uav_coord[0],self.uav_coord[1],self.gcs_coord[0],self.gcs_coord[1],self.uav_alt,self.gcs_alt,self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.uav_heading,self.gcs_heading,self.uav_vel[0],self.uav_vel[1],self.uav_vel[2],self.gcs_vel[0],self.gcs_vel[1],self.gcs_vel[2],self.uav_pose[0],self.uav_pose[1],self.uav_pose[2],self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.goal_mode,self.L, roll_cmd,pitch_cmd,yaw_cmd,thr_cmd,self.voltage,self.current "
                header_text = "log N,time,"+header+" \n"
                f.write(header_text)            
                
                
            #now= datetime.datetime.now()
            #timestamp = now.strftime("%H:%M:%S")
            outstr = str(self.log_n) + ","+ str(self.human_time)+","+ str(dataPkt)+ "\n"
            try:
                f.write(outstr)
            except KeyboardInterrupt:
                print "Keyboard Interrupt Logging"
            return None



##!!!!!!!!!!!!!!!!!!!!!!!!!!!! Run Controller Function !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    def run_sph_pose_controller(self):


        self.control_c +=1

        control_dt = self.dt

        new_state = self.get_relative_angles()
        th = new_state[0]
        phi = new_state[1]
        r = new_state[2]

        print ">> Convention: Theta, Phi, R"
        print ">> Pose:     %.2f, %.2f, %.2f, %.2f" %(th*180/np.pi,phi*180/np.pi,r,self.L)
        print ">> Goal:     %.2f, %.2f, %.2f" %(self.goal_pose[0]*180/np.pi,self.goal_pose[1]*180/np.pi,self.goal_pose[2])
        
        #### START OF SPHERICAL POSITION CONTROLLER #####
        # error_dot
        e_phi_dot =  (r*(self.goal_pose[1]-phi) - self.e_phi)/control_dt
        e_th_dot =   (r*(self.goal_pose[0]-th) - self.e_th)/control_dt
        e_r_dot =    ( (self.goal_pose[2]-r) - self.e_r)/control_dt

        # error
        self.e_phi = r*(self.goal_pose[1] - phi)
        self.e_th = r*(self.goal_pose[0] - th)
        self.e_r = self.goal_pose[2] - r
        
        print ">> Error:    %.2f, %.2f, %.2f" %(self.e_th,self.e_phi,self.e_r)

        # error integration
        self.e_phi_int += self.e_phi*control_dt
        self.e_th_int += self.e_th*control_dt 

        # saturate integration
        self.e_phi_int = self.saturate(self.e_phi_int,-5,5)
        self.e_th_int = self.saturate(self.e_th,-5,5)

        #PID control
        phi_in = (self.k_phi[0]*self.e_phi) +  (self.k_phi[1]*e_phi_dot) + (self.k_phi[2]*self.e_phi_int)
        th_in = (self.k_th[0]*self.e_th) +  (self.k_th[1] * e_th_dot) + (self.k_th[2]*self.e_th_int)
        r_in = self.k_r[0]*self.e_r + self.k_r[1]*e_r_dot 

        print ">> PID Inputs:   %.2f, %.2f, %.2f" %(th_in,phi_in,r_in)

        # Convert spherical PID control to forces in XYZ
        fix = -phi_in*np.sin(phi)*np.sin(th) + th_in*np.cos(th)*np.cos(phi) + (r_in)*np.cos(phi)*np.sin(th)
        fiy = phi_in*np.cos(phi)*np.sin(th) + th_in*np.cos(th)*np.sin(phi) + (r_in)*np.sin(phi)*np.sin(th)
        fiz = -th_in*np.sin(th) + (r_in)*np.cos(th)
        #TODO: Saturate fix, fiy, fiz

        print ">> Convention: X, Y, Z" 
        print ">> PID Forces:     %.2f, %.2f, %.2f" %(fix,fiy,fiz)

        #### Feed Forward Tether Model ####

        #These are the forces that the vehicle needs to exert to balance the tether tension
        # Smart Tether FF uses the tether dynamics to figure out the forces.
        # It is a numerical solution, and not tested on outdoor conditions yet. 
        
        if self.SMART_TETHER:
            [ffx,ffy,ffz] = self.smart_tether_ff(th,phi,r,self.goal_pose[2],self.L)
            print ">> Smart Tether:   %.2f, %.2f, %.2f" %(ffx,ffy,ffz)
            if math.isnan(ffx) or math.isnan(ffy) or math.isnan(ffz):
                print "Smart Tether NaN"
                [ffx,ffy,ffz] = self.basic_tether_ff(th,phi,self.L)
        else:
            [ffx,ffy,ffz] = self.basic_tether_ff(th,phi,self.L) 
            print ">> Basic Tether:   %.2f, %.2f, %.2f" %(ffx,ffy,ffz)
        # Basic tether model uses the weight of the tether and position to determine forces. 
        # It uses trig, and not the ideal tether conditions, but should be more robust than the FF. 

        #Set to zero for simulation
        [ffx,ffy,ffz] = [0,0,0]

        #TODO: Saturate tether model forces. 

        #### Forces from Drag on Vehicle ####
        [fdx, fdy, fdz] = self.get_drag_vector()


        #### Total Forces for Output ####
        ftx = fix + ffx + fdx
        fty = fiy + ffy + fdy
        ftz = fiz + ffz + fdz + self.uav_weight #weight needs to be in newtons

        print ">> Total:    %.2f, %.2f, %.2f" %(ftx,fty,ftz)

        #### Rotate Forces ####
        # This is to keep the front of the vehicle pointed at the ship
        ftx = ftx*np.cos(phi) + fty*np.sin(phi)
        fty = -ftx*np.sin(phi) + fty*np.cos(phi)

        print ">> Total Rotated in BF:    %.2f, %.2f, %.2f" %(ftx,fty,ftz-self.uav_weight)

        f_total = (ftx*ftx+fty*fty+ftz*ftz)**0.5
        pitch = np.arctan(ftx/ftz)
        roll = np.arctan(fty/ftz)

        print ">> Roll, Pitch:     %.2f,   %.2f" %(roll*180/np.pi, pitch*180/np.pi)

        # Saturate
        ATT_MAX = 30*np.pi/180

        #Outputs

        pitch_cmd = self.saturate(pitch,-ATT_MAX,ATT_MAX)
        roll_cmd = self.saturate(roll,-ATT_MAX,ATT_MAX)
        yaw_cmd = self.get_bearing()

        K_THROTTLE = .05
        thr_cmd = ((ftz-self.uav_weight)*K_THROTTLE)+.5
        thr_cmd = self.saturate(thr_cmd,0,1)


        print ">> Output Commands:  %.1f,  %.1f,  %.1f,  %.1f" %(roll_cmd,pitch_cmd,yaw_cmd,thr_cmd)

        #roll_cmd = 0.0 # positive is a roll right. 
        #pitch_cmd = 0.0 # positive is pitch up

        log_data = "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f " %(self.uav_coord[0],self.uav_coord[1],self.gcs_coord[0],self.gcs_coord[1],self.uav_alt,self.gcs_alt,self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.uav_heading,self.gcs_heading,self.uav_vel[0],self.uav_vel[1],self.uav_vel[2],self.gcs_vel[0],self.gcs_vel[1],self.gcs_vel[2],self.uav_pose[0],self.uav_pose[1],self.uav_pose[2],self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.goal_mode,self.L, roll_cmd,pitch_cmd,yaw_cmd,thr_cmd,self.uav_voltage,self.uav_current ) 
        self.write_to_log(log_data)
         
        quat = self.eul2quat(roll_cmd,pitch_cmd,yaw_cmd)

        return [quat[0],quat[1],quat[2],quat[3],thr_cmd]


