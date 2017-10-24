#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import imp
import math
import time
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
        self.uav_heading = 0		# UAV Heading (radians)

        self.gcs_coord = [0,0]   	# GPS Coordinates of GCS [lat,lon] from pixhawk (DD.DDDDDD)
        self.gcs_vel = [0,0,0]		# GCS Velocity [x,y,z] from pixhawk (m/s)
        self.gcs_alt = 0			# GCS Altitude from pixhawk (m)
        self.gcs_heading = 0		# GCS Heading (rad)
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
        self.logging_time = 0 # is a string in other parts of the code
        self.log_file_name = 'log_generic.csv'


        # Controller Gains P D I
        # self.k_phi =[1.2, 2.0,  1.0] 
        # self.k_th  =[1,   2.0,  1.0]
        # self.k_r   =[.5,   3] 

        self.k_phi =[ 0.5, 1,  0.25] # P, D, then I
        self.k_th  =[ 0.5, 1,  0.25] # P, D, then I
        self.k_r   =[0.75, .5      ] # P, D, then I

        self.SMART_TETHER = False

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

        # print '\n\n\nUAV'
        # print '\theading: ',self.uav_heading
        # print '\tcoord: ',self.uav_coord
        # print '\n\nGCS'
        # print '\theading: ',self.gcs_heading
        # print '\tcoord: ',self.gcs_coord
        # print '\n\nDifference: ', self.gcs_heading - self.uav_heading
        # print 'BEARING: ', bearing ,'\n\n\n'

        return bearing#[radians, somehow]

    def get_relative_angles(self): #Tested.
        # Theta First:
        xy_dist = self.get_distance()
        z_dist = self.uav_alt-self.gcs_alt
        theta = np.arctan2(xy_dist,z_dist)
        # Phi Calc
        phi=self.gcs_heading-self.get_bearing()

        # print '\n\n\n'
        # print 'UAV'
        # print '\theading: ',self.uav_heading
        # print '\tcoord: ',self.uav_coord
        # print '\talt: ',self.uav_alt
        # print '\nGCS'
        # print '\theading: ',self.gcs_heading
        # print '\tcoord: ',self.gcs_coord
        # print '\talt: ',self.gcs_alt
        # print '\nBEARING: ', self.get_bearing()
        # print '\nTheta: ', theta*180/np.pi
        # print 'Phi: ', phi
        # print 'r: ', r
        # print '\n\n\n'

        if phi>np.pi:
            phi=phi-2*np.pi


        r = self.get_diagonal_distance()
        self.uav_pose=[theta,phi,r]

        # print '\n\n\nUAV'
        # print '\theading: ',self.uav_heading
        # print '\tcoord: ',self.uav_coord
        # print '\talt: ',self.uav_alt
        # print '\nGCS'
        # print '\theading: ',self.gcs_heading
        # print '\tcoord: ',self.gcs_coord
        # print '\talt: ',self.gcs_alt
        # print '\nBEARING: ', self.get_bearing()
        # print '\nTheta: ', theta*180/np.pi
        # print 'Phi: ', phi
        # print 'r: ', r,'\n\n\n'

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

        # print '\n\n\n g_th,g_phi,L',g_th,g_phi,L
        # print 'Goal pose: ',self.goal_pose,'\n\n\n'

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
            outstr = str(self.log_n) + ","+ str(self.logging_time)+","+ str(dataPkt)+ "\n"
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
        th = new_state[0] # rad
        phi = new_state[1] # rad
        r = new_state[2] # m

        print ">> Pose: Theta %.2f, Phi %.2f, R %.2f, L %.2f" %(th*180/np.pi,phi*180/np.pi,r,self.L)
        print ">> Goal: Theta %.2f, Phi %.2f, R %.2f" %(self.goal_pose[0]*180/np.pi,self.goal_pose[1]*180/np.pi,self.goal_pose[2])
        


        #### START OF SPHERICAL POSITION CONTROLLER #####
        # error_dot
        e_phi_dot =  (r*(self.goal_pose[1]-phi) - self.e_phi)/control_dt
        e_th_dot =   (r*(self.goal_pose[0]-th) - self.e_th)/control_dt
        e_r_dot =    ( (self.goal_pose[2]-r) - self.e_r)/control_dt

        # error
        self.e_phi = r*(self.goal_pose[1] - phi)
        self.e_th = r*(self.goal_pose[0] - th)
        self.e_r = self.goal_pose[2] - r
        
        # print ">> Error: Theta %.2f, Phi %.2f, R %.2f" %(self.e_th,self.e_phi,self.e_r)

        # error integration
        self.e_phi_int += self.e_phi*control_dt
        self.e_th_int += self.e_th*control_dt

        # saturate integration
        self.e_phi_int = self.saturate(self.e_phi_int,-5,5)
        self.e_th_int = self.saturate(self.e_th_int,-5,5)

        #PID control
        phi_in = (self.k_phi[0]*self.e_phi) +  (self.k_phi[1]*e_phi_dot) + (self.k_phi[2]*self.e_phi_int)
        th_in = (self.k_th[0]*self.e_th) +  (self.k_th[1] * e_th_dot) + (self.k_th[2]*self.e_th_int)
        r_in = self.k_r[0]*self.e_r + self.k_r[1]*e_r_dot

        # print ">> PID Inputs: Theta %.2f, Phi %.2f, R %.2f" %(th_in,phi_in,r_in)

        # Convert spherical PID control to forces in XYZ
        r_input_x = r_in*np.sin(th)*np.cos(phi)
        r_input_y = r_in*np.sin(th)*np.sin(phi)
        r_input_z = r_in*np.cos(th)

        phi_input_x = -phi_in*np.sin(phi)
        phi_input_y = phi_in*np.cos(phi)
        phi_input_z = 0

        th_input_x = th_in*np.cos(th)*np.cos(phi)
        th_input_y = th_in*np.cos(th)*np.sin(phi)
        th_input_z = -th_in*np.sin(th)

        fix = r_input_x + phi_input_x + th_input_x
        fiy = r_input_y + phi_input_y + th_input_y
        fiz = r_input_z + phi_input_z + th_input_z
        #TODO: Saturate fix, fiy, fiz

        print ">> PID Forces: X %.2f, Y %.2f, Z %.2f" %(fix,fiy,fiz) #

        #### Feed Forward Tether Model ####

        #These are the forces that the vehicle needs to exert to balance the tether tension
        # Smart Tether FF uses the tether dynamics to figure out the forces.
        # It is a numerical solution, and not tested on outdoor conditions yet. 
        
        if self.SMART_TETHER:
            [ffx,ffy,ffz] = self.smart_tether_ff(th,phi,r,self.goal_pose[2],self.L)
            # print ">> Smart Tether: X %.2f, Y %.2f, Z %.2f" %(ffx,ffy,ffz)
            if math.isnan(ffx) or math.isnan(ffy) or math.isnan(ffz):
                # print "Smart Tether NaN"
                [ffx,ffy,ffz] = self.basic_tether_ff(th,phi,self.L)
        else:
            [ffx,ffy,ffz] = self.basic_tether_ff(th,phi,self.L) 
            # print ">> Basic Tether: X %.2f, Y %.2f, Z %.2f" %(ffx,ffy,ffz)
        # Basic tether model uses the weight of the tether and position to determine forces. 
        # It uses trig, and not the ideal tether conditions, but should be more robust than the FF. 

        # Set to zero for simulation
        # [ffx,ffy,ffz] = [0,0,0]

        #TODO: Saturate tether model forces. 

        #### Forces from Drag on Vehicle ####
        [fdx, fdy, fdz] = self.get_drag_vector() # NOT YET IMPLEMENTED


        #### Total Forces for Output ####
        ftx = fix + ffx + fdx
        fty = fiy + ffy + fdy
        ftz = fiz + ffz + fdz + self.uav_weight # weight needs to be in newtons

        # print ">> Total: X %.2f, Y %.2f, Z %.2f" %(ftx,fty,ftz)

        # The forces have so far been calculated independent of the uav heading. If the uav heading is aligned with the gcs axes (i.e. aligned with the gcs heading), then these forces are correct. This is not always the case, so now rotate the forces to be aligned with the uav heading.

        # Calculate the required difference in heading
        yaw = self.uav_heading # rad
        yaw_goal = self.get_bearing() # rad
        delta_yaw = yaw_goal - yaw

        # Limit delta yaw to be within +- pi from the uav's current heading (ensures uav will turn in optimal direction)
        if delta_yaw < -np.pi:
            delta_yaw = delta_yaw + 2*np.pi
        elif delta_yaw > np.pi:
            delta_yaw = delta_yaw - 2*np.pi

        # Calculate the rotation matrix for the forces. Note that the forces are defined with respect to the gcs reference frame, so the rotation matrix uses the relatice gcs and uav headings.
        relative_heading = -(self.gcs_heading - self.uav_heading)
        R = np.array([[np.cos(relative_heading), -np.sin(relative_heading), 0],\
                      [np.sin(relative_heading),  np.cos(relative_heading), 0],\
                      [                       0,                         0, 1]])
        arr_ft = np.array([ftx, fty, ftz])
        ft_new = R.dot(arr_ft)
        ftx = ft_new[0]
        fty = ft_new[1]
        ftz = ft_new[2]

        # determine pitch and roll. Scale by ftz to add preference to altitude before x,y location
        pitch = np.arctan(ftx/ftz) # rad
        roll = np.arctan(fty/ftz) # rad

        # print ">> Roll %.2f, Pitch %.2f" %(roll*180/np.pi, pitch*180/np.pi)

        # Saturate
        ATT_MAX = 30*np.pi/180

        #Outputs

        pitch_cmd = self.saturate(pitch,-ATT_MAX,ATT_MAX) # rad
        roll_cmd = self.saturate(roll,-ATT_MAX,ATT_MAX) # rad
        yaw_cmd = self.get_bearing() # rad

        # Determine yaw rate based upon required change in yaw (CW positive). Also limit the maximum yaw rate.
        max_yaw_rate = 10 # [deg/s]
        max_yaw_rate = max_yaw_rate*np.pi/180 # [rad/s]
        yaw_rate = self.saturate(delta_yaw,-max_yaw_rate,max_yaw_rate)

        K_THROTTLE = .05
        thr_cmd = ((ftz-self.uav_weight)*K_THROTTLE)+.5
        thr_cmd = self.saturate(thr_cmd,0,1)

        print ">> Output Commands : roll %.1f, pitch %.1f, yaw %.1f, thrust %.1f, yaw_rate %.1f" %(roll_cmd,pitch_cmd,yaw_cmd,thr_cmd, yaw_rate)

        # roll_cmd = 0
        # pitch_cmd = 0
        # yaw_cmd = 90.0
        # thr_cmd = 0.7
        # yaw_rate = 0

        print ">> Actual Commands : roll %.1f, pitch %.1f, yaw %.1f, thrust %.1f, yaw_rate %.1f" %(roll_cmd,pitch_cmd,yaw_cmd,thr_cmd, yaw_rate)

        log_data = "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f " %(self.uav_coord[0],self.uav_coord[1],self.gcs_coord[0],self.gcs_coord[1],self.uav_alt,self.gcs_alt,self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.uav_heading,self.gcs_heading,self.uav_vel[0],self.uav_vel[1],self.uav_vel[2],self.gcs_vel[0],self.gcs_vel[1],self.gcs_vel[2],self.uav_pose[0],self.uav_pose[1],self.uav_pose[2],self.goal_pose[0],self.goal_pose[1],self.goal_pose[2],self.goal_mode,self.L, roll_cmd,pitch_cmd,yaw_cmd,thr_cmd,self.uav_voltage,self.uav_current ) 
        self.write_to_log(log_data)
         
        quat = self.eul2quat(roll_cmd,pitch_cmd,yaw_cmd)

        return [quat[0],quat[1],quat[2],quat[3],thr_cmd,yaw_rate]


