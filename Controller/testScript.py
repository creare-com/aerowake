import numpy as np

point1=[42.356998,-71.092478]
point2=[42.355696,-71.092334]
point3=[42.355805,-71.088758]
point4=[42.357030,-71.090754]

act1=point1
act2=point2

R=6371229

dlat=act1[0]-act2[0]
dlon=act1[1]-act2[1]

a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2
d=R *2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))

print "distance = "+ str(d)

##############################################################
###########################################################

shipHeading=(60)*np.pi/180 #0 to 360 deg


dlat=act1[0]-act2[0]
dlon=act1[1]-act2[1]

arg1= np.sin(dlon*np.pi/180) * np.cos(act2[0]*np.pi/180) 
arg2= np.cos(act1[0]*np.pi/180) * np.sin(act2[0]*np.pi/180) - np.sin(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * np.cos(dlon*np.pi/180)		
gRelAng =  -np.arctan2( arg1, arg2) 

gRelAng= gRelAng-shipHeading

if gRelAng<0:
	gRelAng=gRelAng+2*np.pi


globalAng=gRelAng*180/np.pi


if globalAng<180:
	localAng = (180 - globalAng) + shipHeading*180/np.pi
else:
	localAng = -(360 - globalAng) + shipHeading*180/np.pi

###
phi=360-90
m=160-90

theta = 180- (m - phi)
print str(theta)
#print "global angle   " + str( globalAng)
#print "local angle off ships stern  " + str(localAng)


















